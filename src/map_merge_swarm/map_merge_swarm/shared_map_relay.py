"""
shared_map_relay: gives every robot's Nav2 the merged map.

/shared_map  (frame shared_map, fixed 400x400 grid)
      |  resample with shared_T_<ns>/map from gt_map_tf_node
      v
/<ns>/shared_local  (frame <ns>/map, fixed size, same resolution)

Point each robot's global_costmap static_layer at /<ns>/shared_local.
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

ROBOTS = ["TB3_1", "TB3_2", "TB3_3"]
MARGIN_M = 1.5  # extra room around the shared region, covers small TF drift


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.z * q.w + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))


def compute_extent(shared_info, tx, ty, yaw):
    """
    Bounding box of the whole shared map, expressed in the robot's map frame.
    Computed once, then kept fixed so Nav2 never has to resize its costmap.
    Returns (origin_x, origin_y, width, height) in meters / cells.
    """
    res = shared_info.resolution
    x0 = shared_info.origin.position.x
    y0 = shared_info.origin.position.y
    x1 = x0 + shared_info.width * res
    y1 = y0 + shared_info.height * res

    c, s = math.cos(yaw), math.sin(yaw)
    xs, ys = [], []
    for sx, sy in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        # map = R^T * (shared - t)
        dx, dy = sx - tx, sy - ty
        xs.append(c * dx + s * dy)
        ys.append(-s * dx + c * dy)

    origin_x = math.floor((min(xs) - MARGIN_M) / res) * res
    origin_y = math.floor((min(ys) - MARGIN_M) / res) * res
    width = int(math.ceil((max(xs) + MARGIN_M - origin_x) / res))
    height = int(math.ceil((max(ys) + MARGIN_M - origin_y) / res))
    return origin_x, origin_y, width, height


def resample_to_robot_frame(grid, shared_info, extent, tx, ty, yaw):
    """
    grid: (H, W) int8 array of the shared map, indexed [row, col].
    For every cell of the robot-frame output, look up the shared cell it lands on.
    Cells outside the shared map stay unknown (-1).
    """
    ox, oy, w, h = extent
    res = shared_info.resolution

    mx = ox + (np.arange(w) + 0.5) * res  # cell centers in robot map frame
    my = oy + (np.arange(h) + 0.5) * res
    MX, MY = np.meshgrid(mx, my)  # shape (h, w)

    c, s = math.cos(yaw), math.sin(yaw)
    sx = tx + c * MX - s * MY  # shared = t + R * map
    sy = ty + s * MX + c * MY

    col = np.floor((sx - shared_info.origin.position.x) / res).astype(np.int32)
    row = np.floor((sy - shared_info.origin.position.y) / res).astype(np.int32)
    inside = (
        (col >= 0) & (col < shared_info.width) & (row >= 0) & (row < shared_info.height)
    )

    out = np.full((h, w), -1, dtype=np.int8)
    out[inside] = grid[row[inside], col[inside]]
    return out


class SharedMapRelay(Node):
    def __init__(self):
        super().__init__(
            "shared_map_relay",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._shared = None
        self._grid = None
        self._extent = {}  # ns -> (origin_x, origin_y, width, height), fixed once set

        self.create_subscription(OccupancyGrid, "/shared_map", self._on_shared, qos)
        self._pubs = {
            ns: self.create_publisher(OccupancyGrid, f"/{ns}/shared_local", qos)
            for ns in ROBOTS
        }
        self.create_timer(1.0, self._publish_all)
        self.get_logger().info("shared_map_relay started")

    def _on_shared(self, msg: OccupancyGrid):
        self._shared = msg
        self._grid = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width
        )

    def _publish_all(self):
        if self._shared is None:
            return
        for ns in ROBOTS:
            try:
                tf = self.tf_buffer.lookup_transform("shared_map", f"{ns}/map", Time())
            except Exception as e:
                self.get_logger().warn(
                    f"{ns}: no shared_map <- {ns}/map yet: {e}",
                    throttle_duration_sec=5.0,
                )
                continue

            tx = tf.transform.translation.x
            ty = tf.transform.translation.y
            yaw = yaw_from_quaternion(tf.transform.rotation)

            if ns not in self._extent:
                self._extent[ns] = compute_extent(self._shared.info, tx, ty, yaw)
                self.get_logger().info(f"{ns}: local map extent {self._extent[ns]}")

            out = resample_to_robot_frame(
                self._grid, self._shared.info, self._extent[ns], tx, ty, yaw
            )

            ox, oy, w, h = self._extent[ns]
            msg = OccupancyGrid()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"{ns}/map"
            msg.info.resolution = self._shared.info.resolution
            msg.info.width = w
            msg.info.height = h
            msg.info.origin.position.x = ox
            msg.info.origin.position.y = oy
            msg.info.origin.orientation.w = 1.0
            msg.data = out.ravel().tolist()
            self._pubs[ns].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SharedMapRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
