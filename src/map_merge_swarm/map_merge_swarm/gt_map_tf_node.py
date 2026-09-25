import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from gazebo_msgs.msg import ModelStates
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

ROBOTS = {
    "TB3_1": "burger_1",
    "TB3_2": "burger_2",
    "TB3_3": "burger_3",
}


def yaw_from_quaternion(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))


def se2_inverse(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return (-c * x - s * y, s * x - c * y, -yaw)


def se2_compose(x1, y1, yaw1, x2, y2, yaw2):
    c, s = math.cos(yaw1), math.sin(yaw1)
    return (x1 + c * x2 - s * y2, y1 + s * x2 + c * y2, yaw1 + yaw2)


class GtMapTfNode(Node):
    def __init__(self):
        super().__init__(
            "gt_map_tf_node",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, True),
            ],
        )

        self.get_logger().info("GT Map TF Node Started")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            ModelStates, "/gazebo/model_states", self._on_model_states, qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_broadcaster = TransformBroadcaster(self)

    def _on_model_states(self, msg: ModelStates):
        for robot_ns, robot_name in ROBOTS.items():
            try:
                i = msg.name.index(robot_name)
            except ValueError:
                self.get_logger().warn(
                    f"{robot_name} not in model_states", throttle_duration_sec=2.0
                )
                continue

            try:
                map_to_base = self.tf_buffer.lookup_transform(
                    f"{robot_ns}/map",
                    f"{robot_ns}/base_footprint",
                    Time(),
                )
            except Exception as e:
                self.get_logger().warn(
                    f"TF {robot_ns}/map -> base failed: {e}", throttle_duration_sec=2.0
                )
                continue

            # shared_T_base from Gazebo
            gx = msg.pose[i].position.x
            gy = msg.pose[i].position.y
            gq = msg.pose[i].orientation
            gyaw = yaw_from_quaternion(gq.x, gq.y, gq.z, gq.w)

            # map_T_base from TF
            t = map_to_base.transform.translation
            r = map_to_base.transform.rotation
            mx, my = t.x, t.y
            myaw = yaw_from_quaternion(r.x, r.y, r.z, r.w)

            ix, iy, iyaw = se2_inverse(mx, my, myaw)
            sx, sy, syaw = se2_compose(gx, gy, gyaw, ix, iy, iyaw)

            out = TransformStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = "shared_map"
            out.child_frame_id = f"{robot_ns}/map"
            out.transform.translation.x = sx
            out.transform.translation.y = sy
            out.transform.translation.z = 0.0
            out.transform.rotation.z = math.sin(syaw * 0.5)
            out.transform.rotation.w = math.cos(syaw * 0.5)
            self.tf_broadcaster.sendTransform(out)


def main():
    rclpy.init()
    node = GtMapTfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
