import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseArray
from tf2_ros import Buffer, TransformListener
import math
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

MIN_GOAL_DIST = 1.0  # meters


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

        for robot_ns in self.robot_namespaces:
            self._nav_clients[robot_ns] = ActionClient(
                self, NavigateToPose, f"/{robot_ns}/navigate_to_pose"
            )
            self._busy[robot_ns] = False
            self._cooldown_until[robot_ns] = 0.0  # seconds  (sim time)

    def _get_robot_pose(self, robot_ns):
        t = self.tf_buffer.lookup_transform(
            "shared_map",
            f"{robot_ns}/base_footprint",
            Time(),
        )
        return t.transform.translation.x, t.transform.translation.y

    def _nearest_frontier_for_ns(self, frontiers, idle, min_d2):

        best = None
        for i, (ns, rx, ry) in enumerate(idle):
            for j, (fx, fy) in enumerate(frontiers):
                d2 = (fx - rx) ** 2 + (fy - ry) ** 2
                if d2 < min_d2:  # too close → ignore
                    continue
                if best is None or d2 < best[0]:
                    best = (d2, i, j)
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
        now = self.get_clock().now().nanoseconds * 1e-9
        return (not self._busy[ns]) and (now >= self._cooldown_until[ns])

    def _send_goal(self, ns, pose_robot):
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = f"{ns}/map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = pose_robot

        self._busy[ns] = True
        send_future = self._nav_clients[ns].send_goal_async(goal)
        send_future.add_done_callback(
            lambda future, robot=ns: self._on_goal_response(future, robot)
        )

    def _on_goal_response(self, future, ns):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"{ns}: goal rejected")
            self._busy[ns] = False
            return
        self.get_logger().info(f"{ns}: goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, robot=ns: self._on_result(future, robot)
        )

    def _on_result(self, future, ns):
        status = future.result().status
        self.get_logger().info(f"{ns}: nav finished status = {status}")
        self._busy[ns] = False
        self._cooldown_until[ns] = self.get_clock().now().nanoseconds * 1e-9 + 3.0

    def _on_frontier(self, msg: PoseArray):
        if not msg.poses:
            return

        idle = []
        for ns in self.robot_namespaces:
            if not self._is_idle(ns):
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
        while idle and frontiers:
            best = self._nearest_frontier_for_ns(frontiers, idle, min_d2)
            if best is None:
                self.get_logger().info(
                    "no frontier far enough for idle robots",
                    throttle_duration_sec=2.0,
                )
                break

            _, i, j = best
            ns, rx, ry = idle.pop(i)
            frontier_xy = frontiers.pop(j)

            backoff = self._backoff_goal(frontier_xy, (rx, ry))
            gx, gy = backoff
            dist_goal = math.sqrt((gx - rx) ** 2 + (gy - ry) ** 2)
            if dist_goal < 0.5:
                idle.append((ns, rx, ry))
                continue
            try:
                pose_robot = self._transform_pose_to_robot_frame(backoff, ns)
            except Exception as e:
                self.get_logger().warn(
                    f"{ns}: TF failed: {e}", throttle_duration_sec=2.0
                )
                idle.append((ns, rx, ry))
                frontiers.append(frontier_xy)
                continue

            if not self._nav_clients[ns].wait_for_server(timeout_sec=0.0):
                self.get_logger().warn(
                    f"{ns}: Nav2 not ready", throttle_duration_sec=2.0
                )
                idle.append((ns, rx, ry))
                frontiers.append(frontier_xy)
                continue
            self.get_logger().info(
                f"assign {ns} -> frontier {frontier_xy}",
                throttle_duration_sec=1.0,
            )
            self._send_goal(ns, pose_robot)


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
