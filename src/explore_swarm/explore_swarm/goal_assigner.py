import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from scipy import ndimage
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener

MIN_GOAL_DIST = 0.5  # meters
OCCUPIED_THRESH = 50

CLAIM_RADIUS = 1.2  # meters; only matters if same open space
IDLE_ESCAPE_S = 10.0

BLACKLIST_RADIUS = 0.6  # meters, bigger than the 0.9 m goal-to-frontier offset
HOLD_ABORTED_S = 60.0  # Nav2 status 6
HOLD_CANCELLED_S = 90.0  # Nav2 status 5 (the stuck watchdog will cause these)

STUCK_S = 20.0
STUCK_MOVE_M = 0.3
MAX_STUCK_COUNT = 3

GAP_PENALTY = 1.0  # meters; penalty for crossing into an unmapped blob
DISPERSE_W = 0.5  # weight: how strongly robots push away from each other
DISPERSE_RANGE = 6.0  # max distance to consider teammate repulsion


class GoalAssigner(Node):
    def __init__(self):
        super().__init__(
            "goal_assigner",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, value=True)
            ],
        )
        self.get_logger().info("Goal assigner node initialized successfully")

        self.create_subscription(PoseArray, "/frontiers", self._on_frontier, 10)
        self.get_logger().info("goal_assigner listening on /frontiers")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_namespaces = ["TB3_1", "TB3_2", "TB3_3"]
        self._nav_clients = {}

        self._busy = {}
        self._cooldown_until = {}

        self._shared_map = None
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            OccupancyGrid, "/shared_map", self._on_shared_map, map_qos
        )

        for robot_ns in self.robot_namespaces:
            self._nav_clients[robot_ns] = ActionClient(
                self, NavigateToPose, f"/{robot_ns}/navigate_to_pose"
            )
            self._busy[robot_ns] = False
            self._cooldown_until[robot_ns] = 0.0  # seconds  (sim time)

        self._claims = {}
        self._free_labels = None

        self._clearance = None  # meters to nearest wall, same shape as the map
        self._unassigned_since = {}  # ns -> sim time it was first left without a goal

        self._blacklist = []  # entries: (x, y, expires_at) in shared_map meters and sim seconds

        self._goal_handles = {}  # ns -> active goal handle
        self._last_move = {}  # ns -> (x, y, time_s) when it last made progress
        self._stuck_count = {}  # ns -> how many times it has been stuck in a row
        self._visit_counts = {}  # (x, y) -> count of completed visits
        self.create_timer(5.0, self._watchdog)

    def _on_shared_map(self, msg):
        self._shared_map = msg
        grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )
        self._free_labels, _ = ndimage.label(
            grid == 0
        )  # each blob of connected free cells gets a number
        occupied = grid >= OCCUPIED_THRESH
        self._clearance = (
            ndimage.distance_transform_edt(~occupied) * msg.info.resolution
        )

    def _labels_near(self, cell, r=3):
        i, j = cell
        patch = self._free_labels[
            max(0, j - r) : j + r + 1, max(0, i - r) : i + r + 1
        ]  # [row=j, col=i]
        return set(np.unique(patch)) - {0}

    def _reachable_via_free(self, robot_xy, goal_xy):
        if self._free_labels is None:
            return False
        rc = self._world_to_cell(*robot_xy)
        gc = self._world_to_cell(*goal_xy)
        if rc is None or gc is None:
            return False
        return bool(self._labels_near(rc) & self._labels_near(gc))

    def _world_to_cell(self, x, y):
        if self._shared_map is None:
            return None
        info = self._shared_map.info
        i = int((x - info.origin.position.x) / info.resolution)
        j = int((y - info.origin.position.y) / info.resolution)
        if i < 0 or i >= info.width or j < 0 or j >= info.height:
            return None
        return (i, j)

    def _cell_value(self, i, j):
        if self._shared_map is None:
            return None
        if (
            i < 0
            or i >= self._shared_map.info.width
            or j < 0
            or j >= self._shared_map.info.height
        ):
            return None
        return self._shared_map.data[j * self._shared_map.info.width + i]

    def _line_crosses_occupied(self, x0, y0, x1, y1):
        """
        Walk in small steps from (x0, y0) to (x1, y1).
        Return True if any sample lands on an occupied (wall) cell.
        """
        if self._shared_map is None:
            return True

        dx = x1 - x0
        dy = y1 - y0
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 1e-3:
            return False

        res = self._shared_map.info.resolution  # meters per cell
        step = res / 2.0  # half a cell
        # number of steps to take along the line
        n_steps = max(1, int(dist / step))

        for step_idx in range(n_steps + 1):
            fraction = step_idx / n_steps  # 0.0 at start, 1.0 at end

            x = x0 + fraction * dx
            y = y0 + fraction * dy

            cell = self._world_to_cell(x, y)
            if cell is None:
                return True  # off the map → treat as blocked

            value = self._cell_value(cell[0], cell[1])
            if value is None or value >= OCCUPIED_THRESH:
                return True  # wall (or invalid)
        return False  # never hit a wall

    def _frontier_claimed_by_other(self, fx, fy, robot_ns):
        for other_ns, (cx, cy) in self._claims.items():
            if other_ns == robot_ns:
                continue
            if math.hypot(fx - cx, fy - cy) > CLAIM_RADIUS:
                continue
            if not self._line_crosses_occupied(fx, fy, cx, cy):
                return True
        return False

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _is_blacklisted(self, x, y):
        now = self._now()
        self._blacklist = [
            b for b in self._blacklist if b[2] > now
        ]  # forget expired entries

        for bx, by, expires_at in self._blacklist:
            if math.hypot(x - bx, y - by) < BLACKLIST_RADIUS:
                return True
        return False

    def _get_robot_pose(self, robot_ns):
        t = self.tf_buffer.lookup_transform(
            "shared_map",
            f"{robot_ns}/base_footprint",
            Time(),
        )
        return t.transform.translation.x, t.transform.translation.y

    def _nearest_frontier_for_ns(self, frontiers, idle, min_d2, rejected):
        best = None
        stats = {"close": 0, "claimed": 0, "rejected": 0, "blacklisted": 0}
        for i, (ns, rx, ry) in enumerate(idle):
            for j, (fx, fy) in enumerate(frontiers):
                d2 = (fx - rx) ** 2 + (fy - ry) ** 2
                if d2 < min_d2:  # too close → ignore
                    stats["close"] += 1
                    continue
                if (fx, fy) in rejected[ns]:
                    stats["rejected"] += 1
                    continue
                if self._frontier_claimed_by_other(fx, fy, ns):
                    stats["claimed"] += 1
                    continue
                if self._is_blacklisted(fx, fy):
                    stats["blacklisted"] += 1
                    continue
                if best is None or d2 < best[0]:
                    best = (d2, i, j)
        if best is None:
            self.get_logger().info(
                f"no pick: {stats} idle={[n for n, _, _ in idle]} claims={self._claims}",
                throttle_duration_sec=2.0,
            )
        return best

    def _backoff_goal(self, frontier_xy, robot_xy):
        vx = robot_xy[0] - frontier_xy[0]
        vy = robot_xy[1] - frontier_xy[1]
        length = math.sqrt(vx**2 + vy**2)
        if length < 1e-3:
            return frontier_xy

        ux = vx / length
        uy = vy / length

        # walk toward robot, but not more than (length - 0.1)
        step = min(0.4, max(0.0, length - 0.1))
        if step < 0.05:
            return frontier_xy
        return (frontier_xy[0] + step * ux, frontier_xy[1] + step * uy)

    def _transform_pose_to_robot_frame(self, backoff_frontier, ns):
        pose_shared = PoseStamped()
        pose_shared.header.frame_id = "shared_map"
        pose_shared.header.stamp = Time().to_msg()
        pose_shared.pose.position.x = backoff_frontier[0]
        pose_shared.pose.position.y = backoff_frontier[1]
        pose_shared.pose.orientation.w = 1.0

        # transform to robot frame
        tf = self.tf_buffer.lookup_transform(f"{ns}/map", "shared_map", Time())
        pose_robot = do_transform_pose(pose_shared.pose, tf)
        return pose_robot

    def _is_idle(self, ns):
        now = self._now()
        if self._stuck_count.get(ns, 0) >= MAX_STUCK_COUNT:
            if now < self._cooldown_until.get(ns, 0.0):
                return False
            self._stuck_count[ns] = 0
        return (not self._busy[ns]) and (now >= self._cooldown_until[ns])

    def _send_goal(self, ns, pose_robot, claim_xy):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = f"{ns}/map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = pose_robot

        self._busy[ns] = True
        self._claims[ns] = claim_xy
        send_future = self._nav_clients[ns].send_goal_async(goal)
        send_future.add_done_callback(
            lambda future, robot=ns: self._on_goal_response(future, robot)
        )

    def _on_goal_response(self, future, ns):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"{ns}: goal rejected")
            self._busy[ns] = False
            self._claims.pop(ns, None)
            return
        self.get_logger().info(f"{ns}: goal accepted")
        self._goal_handles[ns] = goal_handle
        try:
            rx, ry = self._get_robot_pose(ns)
            self._last_move[ns] = (rx, ry, self._now())
        except Exception:
            self._last_move[ns] = None

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, robot=ns: self._on_result(future, robot)
        )

    def _on_result(self, future, ns):
        claim = self._claims.pop(ns, None)

        self._goal_handles.pop(ns, None)
        self._last_move.pop(ns, None)

        status = future.result().status
        now = self._now()

        if status == 4:
            self._stuck_count[ns] = 0
            if claim is not None:
                cx, cy = claim

                # Check if we already visited this frontier before (within 0.8m)
                matching_entry = None
                for (vx, vy), count in self._visit_counts.items():
                    if math.hypot(cx - vx, cy - vy) < 0.8:
                        matching_entry = (vx, vy)
                        break

                if matching_entry is None:
                    # Attempt 1: First visit. Give a 60s cooldown so others don't rush here.
                    self._visit_counts[(cx, cy)] = 1
                    self._blacklist.append((cx, cy, now + 60.0))
                else:
                    count = self._visit_counts[matching_entry] + 1
                    self._visit_counts[matching_entry] = count
                    if count >= 2:
                        # Attempt 2 finished: It's an unreachable shadow (trash can/pillar)!
                        # Blacklist permanently for 10 minutes (600s)!
                        self._blacklist.append((cx, cy, now + 600.0))
                        self.get_logger().info(
                            f"{ns}: frontier at ({cx:.2f},{cy:.2f}) visited {count} times without clearing. Blacklisting 600s!"
                        )
                    else:
                        self._blacklist.append((cx, cy, now + 60.0))

        if status != 4 and claim is not None:
            if status == 6:
                hold = HOLD_ABORTED_S
            else:
                hold = HOLD_CANCELLED_S
            self._blacklist.append((claim[0], claim[1], now + hold))
            self.get_logger().info(
                f"{ns}: blacklist ({claim[0]:.2f},{claim[1]:.2f}) for {hold:.0f}s"
            )

        self.get_logger().info(f"{ns}: nav finished status = {status}")
        self._busy[ns] = False
        self._cooldown_until[ns] = now + 0.1

    def _has_clearance(self, gx, gy, radius_m):
        cell = self._world_to_cell(gx, gy)
        if cell is None or self._clearance is None:
            return False
        return bool(self._clearance[cell[1], cell[0]] >= radius_m)  # [row=j, col=i]

    def _is_goal_valid(self, frontier_xy, goal_xy, robot_xy, clearance_m=0.3):
        """True if Nav2 end point is free and same side of walls as frontier."""
        gx, gy = goal_xy
        fx, fy = frontier_xy
        cell = self._world_to_cell(gx, gy)
        if cell is None:
            return False
        value = self._cell_value(*cell)
        if value is None or value >= OCCUPIED_THRESH:  # Step B: unknown is OK
            self.get_logger().info(
                f"reject: goal cell value={value}", throttle_duration_sec=1.0
            )
            return False
        if self._line_crosses_occupied(fx, fy, gx, gy):
            self.get_logger().info(
                "reject: frontier->goal crosses wall", throttle_duration_sec=1.0
            )
            return False
        if not self._has_clearance(gx, gy, clearance_m):
            self.get_logger().info(
                "reject: too close to obstacle", throttle_duration_sec=1.0
            )
            return False

        # NOTE: Do not reject goals across unmapped doorways with _reachable_via_free!
        # Pattern 5 sort_key already explores local room first.
        # Once local room is done, Nav2 (with allow_unknown: true) will route through doors.
        # if not self._reachable_via_free(robot_xy, goal_xy):
        #     self.get_logger().info(
        #         "reject: no known free route", throttle_duration_sec=1.0
        #     )
        #     return False
        return True

    def _goal_candidates(self, frontier_xy, robot_xy):
        fx, fy = frontier_xy
        rx, ry = robot_xy
        cands = []
        vx, vy = rx - fx, ry - fy
        length = math.hypot(vx, vy)
        if length > 1e-3:
            ux, uy = vx / length, vy / length
            for step in (
                0.4,
                0.3,
                0.6,
                0.9,
            ):  # toward the robot, 0.4 is the old backoff
                if step < length - 0.1:
                    cands.append((fx + step * ux, fy + step * uy))
        ring = []
        for radius in (0.4, 0.7):  # points around the frontier
            for k in range(8):
                a = k * math.pi / 4
                ring.append((fx + radius * math.cos(a), fy + radius * math.sin(a)))
        ring.sort(
            key=lambda p: (p[0] - rx) ** 2 + (p[1] - ry) ** 2
        )  # closest to robot first
        return cands + ring

    def _find_goal(self, frontier_xy, robot_xy):
        cands = self._goal_candidates(frontier_xy, robot_xy)
        for clearance in (0.32, 0.22):  # strict first, relaxed for narrow spots
            for cand in cands:
                if self._is_goal_valid(frontier_xy, cand, robot_xy, clearance):
                    return cand
        return None

    def _escape_goal(self, rx, ry, radius_m=2.0, min_move_m=0.5, min_clear_m=0.25):
        if self._free_labels is None or self._clearance is None:
            return None
        rc = self._world_to_cell(rx, ry)
        if rc is None:
            return None
        ri, rj = rc
        info = self._shared_map.info
        res = info.resolution
        r = int(radius_m / res)
        j0, j1 = max(0, rj - r), min(info.height, rj + r + 1)
        i0, i1 = max(0, ri - r), min(info.width, ri + r + 1)

        labels = self._free_labels[j0:j1, i0:i1]
        clear = self._clearance[j0:j1, i0:i1]
        jj, ii = np.mgrid[j0:j1, i0:i1]
        dist = np.hypot(ii - ri, jj - rj) * res

        my_labels = list(self._labels_near(rc))
        if not my_labels:
            return None

        for c_thresh in (min_clear_m, 0.18):  # strict first, then relaxed
            mask = (
                np.isin(labels, my_labels)
                & (clear >= c_thresh)
                & (dist >= min_move_m)
                & (dist <= radius_m)
            )
            if not mask.any():
                continue

            # Sort candidate cells by clearance (highest clearance first)
            cand_indices = np.argwhere(mask)
            cand_indices = sorted(
                cand_indices, key=lambda idx: clear[idx[0], idx[1]], reverse=True
            )

            for cj, ci in cand_indices:
                gx = info.origin.position.x + (i0 + ci + 0.5) * res
                gy = info.origin.position.y + (j0 + cj + 0.5) * res

                if self._is_blacklisted(gx, gy):
                    continue

                if any(
                    math.hypot(gx - cx, gy - cy) < CLAIM_RADIUS
                    for cx, cy in self._claims.values()
                ):
                    continue
                return (gx, gy)

        return None

    def _try_escape(self, ns, rx, ry):
        goal = self._escape_goal(rx, ry)
        if goal is None:
            self.get_logger().info(f"{ns}: no escape goal", throttle_duration_sec=5.0)
            return
        if not self._nav_clients[ns].wait_for_server(timeout_sec=0.0):
            return
        try:
            pose_robot = self._transform_pose_to_robot_frame(goal, ns)
        except Exception as e:
            self.get_logger().warn(f"{ns}: TF failed: {e}", throttle_duration_sec=2.0)
            return
        self.get_logger().info(f"{ns}: escape goal ({goal[0]:.2f},{goal[1]:.2f})")
        self._send_goal(ns, pose_robot, goal)

    def _watchdog(self):
        now = self._now()
        for ns in self.robot_namespaces:
            if not self._busy.get(ns, False):
                continue
            handle = self._goal_handles.get(ns)
            if handle is None:
                continue

            last_data = self._last_move.get(ns)
            if last_data is None:
                try:
                    rx, ry = self._get_robot_pose(ns)
                    self._last_move[ns] = (rx, ry, now)
                except Exception:
                    pass
                continue
            last_x, last_y, last_time = last_data

            try:
                rx, ry = self._get_robot_pose(ns)
            except Exception:
                continue

            distance_moved = math.hypot(rx - last_x, ry - last_y)
            if distance_moved >= STUCK_MOVE_M:
                self._last_move[ns] = (rx, ry, now)

            elif now - last_time > STUCK_S:
                count = self._stuck_count.get(ns, 0) + 1
                self._stuck_count[ns] = count
                self.get_logger().warn(
                    f"{ns}: STUCK! No progress for {STUCK_S:.0f}s (streak: {count}). Cancelling goal..."
                )
                handle.cancel_goal_async()
                self._goal_handles.pop(ns, None)

    def _on_frontier(self, msg: PoseArray):
        if not msg.poses:
            return

        idle = []
        for ns in self.robot_namespaces:
            if not self._is_idle(ns):
                self._unassigned_since.pop(ns, None)
                continue
            try:
                rx, ry = self._get_robot_pose(ns)
            except Exception as e:
                self.get_logger().warn(
                    f"{ns}: TF failed: {e}", throttle_duration_sec=2.0
                )
                continue
            idle.append((ns, rx, ry))

        if not idle:
            return

        frontiers = [(p.position.x, p.position.y) for p in msg.poses]
        min_d2 = MIN_GOAL_DIST**2
        rejected = {
            ns: set() for ns in self.robot_namespaces
        }  # per robot, this callback only

        all_poses = {}
        for r_name in self.robot_namespaces:
            try:
                all_poses[r_name] = self._get_robot_pose(r_name)
            except Exception:
                pass

        for ns, rx, ry in idle:
            if not frontiers:
                break

            # sort robots by distance to this robot
            #  (Optional bonus: sort by (not own_room, distance) so local frontiers come first)

            rc = self._world_to_cell(rx, ry)
            robot_labels = self._labels_near(rc) if rc else set()

            def sort_key(f):
                fx, fy = f
                d_self = math.hypot(fx - rx, fy - ry)

                fc = self._world_to_cell(fx, fy)
                f_labels = self._labels_near(fc) if fc else set()
                same_blob = bool(robot_labels & f_labels)
                gap_cost = 0.0 if same_blob else GAP_PENALTY
                # Penalty if straight line crosses a wall (forces robot to finish its own room first)
                wall_cost = 2.5 if self._line_crosses_occupied(rx, ry, fx, fy) else 0.0

                # Only penalize if another robot is already close to this frontier (< 2.5m)
                other_dists = [
                    math.hypot(fx - ox, fy - oy)
                    for o_ns, (ox, oy) in all_poses.items()
                    if o_ns != ns
                ]
                d_other = min(other_dists) if other_dists else 10.0

                # Positive penalty: steers robots away from where teammates are working
                teammate_penalty = max(0.0, 2.5 - d_other) * 2.0

                return d_self + gap_cost + wall_cost + teammate_penalty

            sorted_frontiers = sorted(frontiers, key=sort_key)

            # find first valid frontier for this robot
            assigned = False
            for f_xy in sorted_frontiers:
                fx, fy = f_xy

                if (fx - rx) ** 2 + (fy - ry) ** 2 < min_d2:
                    continue
                if self._frontier_claimed_by_other(fx, fy, ns):
                    continue
                if self._is_blacklisted(fx, fy):
                    continue

                d_self = math.hypot(fx - rx, fy - ry)
                other_dists = [
                    math.hypot(fx - ox, fy - oy)
                    for o_ns, (ox, oy) in all_poses.items()
                    if o_ns != ns
                ]
                d_other = min(other_dists) if other_dists else 10.0
                if d_other < 1.2 and d_self > 2.5:
                    continue

                goal = self._find_goal(f_xy, (rx, ry))
                if goal is None or math.hypot(goal[0] - rx, goal[1] - ry) < 0.2:
                    continue

                # transform to robot frame
                try:
                    pose_robot = self._transform_pose_to_robot_frame(goal, ns)
                except Exception as e:
                    self.get_logger().warn(
                        f"{ns}: TF failed: {e}", throttle_duration_sec=2.0
                    )
                    continue

                if not self._nav_clients[ns].wait_for_server(timeout_sec=0.0):
                    self.get_logger().warn(
                        f"{ns}: Nav2 not ready", throttle_duration_sec=2.0
                    )
                    continue

                self.get_logger().info(f"assign {ns} -> frontier ({fx:.2f},{fy:.2f})")
                self._send_goal(ns, pose_robot, goal)
                self._unassigned_since.pop(ns, None)
                frontiers.remove(f_xy)
                assigned = True
                break

            if not assigned:
                self.get_logger().info(
                    f"{ns}: no valid frontier found this cycle",
                    throttle_duration_sec=2.0,
                )
                now = self._now()
                t0 = self._unassigned_since.setdefault(ns, now)
                if now - t0 > IDLE_ESCAPE_S:
                    self._try_escape(ns, rx, ry)
                    self._unassigned_since[ns] = now


def main(args=None):
    rclpy.init(args=args)
    node = GoalAssigner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
