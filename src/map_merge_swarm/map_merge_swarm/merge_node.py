import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener
import math
from rclpy.time import Time

SHARED_RES = 0.05
SHARED_WIDTH = 400
SHARED_HEIGHT = 400
SHARED_ORIGIN_X = -10.0
SHARED_ORIGIN_Y = -10.0


class MergeNode(Node):
    def __init__(self):
        super().__init__(
            "map_merge_node",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True),
            ],
        )

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.latest_maps = {"TB3_1": None, "TB3_2": None, "TB3_3": None}

        for robot_ns in self.latest_maps:
            self.create_subscription(
                OccupancyGrid,
                f"/{robot_ns}/map",
                self._make_map_callback(robot_ns),
                map_qos,
            )

        self.shared_map_publisher = self.create_publisher(
            OccupancyGrid, "/shared_map", map_qos
        )
        self.timer = self.create_timer(1.0, self.merge_and_publish)

        self.get_logger().info("map_merge_node started")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _make_map_callback(self, robot_ns):
        def callback(msg):
            self.latest_maps[robot_ns] = msg

        return callback

    def merge_and_publish(self):
        canvas = [-1] * SHARED_WIDTH * SHARED_HEIGHT
        for robot_ns, msg in self.latest_maps.items():
            if msg is None:
                continue
            self._paint_robot_map(canvas, robot_ns, msg)

        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "shared_map"
        out.info.resolution = SHARED_RES
        out.info.width = SHARED_WIDTH
        out.info.height = SHARED_HEIGHT
        out.info.origin.position.x = SHARED_ORIGIN_X
        out.info.origin.position.y = SHARED_ORIGIN_Y
        out.info.origin.position.z = 0.0
        out.info.origin.orientation.w = 1.0
        out.data = canvas
        self.shared_map_publisher.publish(out)

    def _fuse_maps(self, old, new):
        if new == 100 or old == 100:
            return 100
        if old == 0 or new == 0:
            return 0
        return -1

    def _shared_to_index(self, shared_x, shared_y):
        col = int((shared_x - SHARED_ORIGIN_X) / SHARED_RES)
        row = int((shared_y - SHARED_ORIGIN_Y) / SHARED_RES)
        if col < 0 or col >= SHARED_WIDTH or row < 0 or row >= SHARED_HEIGHT:
            return None
        return row * SHARED_WIDTH + col

    def _paint_robot_map(self, canvas, robot_ns, msg):
        try:
            tf = self.tf_buffer.lookup_transform(
                "shared_map", f"{robot_ns}/map", Time()
            )
        except Exception as e:
            self.get_logger().warn(
                f"No shared_map <- {robot_ns}/map: {e}", throttle_duration_sec=2.0
            )
            return

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.z * q.w + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))
        c, s = math.cos(yaw), math.sin(yaw)

        res = msg.info.resolution
        width = msg.info.width
        height = msg.info.height
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        for row in range(height):
            for col in range(width):
                value = int(msg.data[row * width + col])
                if value == -1:
                    continue

                # cell center in the robot's private map frame
                map_x = origin_x + (col + 0.5) * res
                map_y = origin_y + (row + 0.5) * res

                # apply shared_T_map
                shared_x = tx + c * map_x - s * map_y
                shared_y = ty + s * map_x + c * map_y
                idx = self._shared_to_index(shared_x, shared_y)
                if idx is not None:
                    canvas[idx] = self._fuse_maps(canvas[idx], value)


def main(args=None):
    rclpy.init(args=args)
    node = MergeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
