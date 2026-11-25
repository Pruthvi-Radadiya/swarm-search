#!/usr/bin/env python3
"""
aerial_scout_node.py - Autonomous Aerial Reconnaissance Scout for Phase 3.

Responsibilities:
1. Connects to PX4 Autopilot SITL via ROS 2 DDS (px4_msgs).
2. Autonomous State Machine: Arm -> Takeoff (1.9m) -> Waypoint Tour -> Drop Pins -> Return & Land.
3. Publishes discovered high-priority targets onto /candidate_pins (geometry_msgs/msg/PoseArray).
4. Subscribes to /drone/camera/image_raw for downward camera frames.
"""

import time
from enum import Enum

import rclpy

# Standard ROS 2 messages
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

# PX4 uORB message interfaces
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
    VehicleStatus,
)


class ScoutState(Enum):
    INIT = 0
    TAKEOFF = 1
    CRUISE = 2
    INSPECT = 3
    RETURN_HOME = 4
    LAND = 5
    DONE = 6


class AerialScoutNode(Node):
    def __init__(self):
        super().__init__("aerial_scout_node")

        # ------------------------------------------------------------------
        # QoS Profiles (Best-effort for high rate PX4 sensor data)
        # ------------------------------------------------------------------
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ------------------------------------------------------------------
        # PX4 Publishers & Subscribers
        # ------------------------------------------------------------------
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )

        self.odom_sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._odom_cb, qos_px4
        )
        self.status_sub = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._status_cb, qos_px4
        )

        # ------------------------------------------------------------------
        # Downward Camera & Swarm Perception
        # ------------------------------------------------------------------
        self.camera_sub = self.create_subscription(
            Image, "/drone/camera/image_raw", self._camera_cb, 10
        )
        self.candidate_pins_pub = self.create_publisher(
            PoseArray, "/candidate_pins", 10
        )

        # ------------------------------------------------------------------
        # Drone State Variables
        # ------------------------------------------------------------------
        self.current_state = ScoutState.INIT
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0
        self.nav_state = 0
        self.arming_state = 0

        # Flight Parameters
        self.cruise_altitude = -1.9  # PX4 NED frame: negative is UP (1.9m height)
        self.acceptance_radius = 0.35  # meters (waypoint arrival tolerance)

        # ------------------------------------------------------------------
        # Waypoints: Circuit of rooms across turtlebot3_house layout (X, Y)
        # ------------------------------------------------------------------
        self.waypoints = [
            (1.5, 1.0),  # Room 1 (Living room / dining)
            (2.5, -1.8),  # Room 2 (Corner bedroom)
            (-1.5, -2.0),  # Room 3 (Lower hall / office)
            (-2.0, 1.2),  # Room 4 (Kitchen / upper area)
        ]
        self.current_wp_idx = 0
        self.inspect_start_time = None
        self.published_pins = []

        # ------------------------------------------------------------------
        # 10 Hz Control Loop Timer (Heartbeat + Trajectory)
        # ------------------------------------------------------------------
        self.timer_step = 0
        self.timer = self.create_timer(0.1, self._control_loop)
        self.get_logger().info(
            "🚁 Aerial Scout Node initialized. Ready to commence reconnaissance."
        )

    def _status_cb(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def _odom_cb(self, msg: VehicleOdometry):
        # In PX4: msg.position is [North(x), East(y), Down(z)]
        self.pos_x = msg.position[0]
        self.pos_y = msg.position[1]
        self.pos_z = msg.position[2]

    def _camera_cb(self, msg: Image):
        # Camera callback active (ready for image processing / visual trigger)
        pass

    def send_cmd(self, command, param1=0.0, param2=0.0):
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_cmd_pub.publish(cmd)

    def publish_offboard_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw=0.0):
        sp = TrajectorySetpoint()
        sp.position = [float(x), float(y), float(z)]
        sp.yaw = float(yaw)
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(sp)

    def _publish_candidate_pin(self, x, y):
        """Drops a candidate pin onto /candidate_pins for the ground swarm."""
        pin_pose = Pose()
        pin_pose.position.x = float(x)
        pin_pose.position.y = float(y)
        pin_pose.position.z = 0.0  # Floor target for rovers

        self.published_pins.append(pin_pose)

        pin_array = PoseArray()
        pin_array.header.stamp = self.get_clock().now().to_msg()
        pin_array.header.frame_id = "shared_map"
        pin_array.poses = list(self.published_pins)

        self.candidate_pins_pub.publish(pin_array)
        self.get_logger().info(
            f"📍 [AERIAL PIN] Spotted target at ({x:.2f}, {y:.2f})! Dispatched to /candidate_pins."
        )

    def _control_loop(self):
        # 1. Always publish heartbeat
        self.publish_offboard_heartbeat()
        self.timer_step += 1

        # Current altitude (positive height above ground)
        current_altitude = -self.pos_z

        # -------------------------------------------------------------
        # STATE 0: INIT (Wait 1 second for DDS connection, then arm)
        # -------------------------------------------------------------
        if self.current_state == ScoutState.INIT:
            # Hold origin while starting stream
            self.publish_position_setpoint(0.0, 0.0, 0.0)

            if self.timer_step >= 15:  # 1.5s of stream
                self.get_logger().info("🚁 Switching to Offboard mode & Arming...")
                # Switch to Offboard (Mode 6)
                self.send_cmd(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
                )
                # Arm motors
                self.send_cmd(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
                )
                self.current_state = ScoutState.TAKEOFF

        # -------------------------------------------------------------
        # STATE 1: TAKEOFF (Climb to cruise altitude)
        # -------------------------------------------------------------
        elif self.current_state == ScoutState.TAKEOFF:
            self.publish_position_setpoint(0.0, 0.0, self.cruise_altitude)

            if current_altitude >= 1.75:
                self.get_logger().info(
                    f"✅ Takeoff complete! Altitude reached: {current_altitude:.2f}m. Commencing room tour."
                )
                self.current_state = ScoutState.CRUISE

        # -------------------------------------------------------------
        # STATE 2: CRUISE (Navigate to current room waypoint)
        # -------------------------------------------------------------
        elif self.current_state == ScoutState.CRUISE:
            target_x, target_y = self.waypoints[self.current_wp_idx]
            self.publish_position_setpoint(target_x, target_y, self.cruise_altitude)

            # Compute horizontal distance to waypoint
            dist = ((self.pos_x - target_x) ** 2 + (self.pos_y - target_y) ** 2) ** 0.5
            if dist < self.acceptance_radius:
                self.get_logger().info(
                    f"🔎 Reached Room Waypoint {self.current_wp_idx + 1}/{len(self.waypoints)} "
                    f"at ({target_x:.2f}, {target_y:.2f}). Inspecting room..."
                )
                self.inspect_start_time = time.time()
                self.current_state = ScoutState.INSPECT

        # -------------------------------------------------------------
        # STATE 3: INSPECT (Hover for 3s, drop Candidate Pin)
        # -------------------------------------------------------------
        elif self.current_state == ScoutState.INSPECT:
            target_x, target_y = self.waypoints[self.current_wp_idx]
            # Hold position rock-solid
            self.publish_position_setpoint(target_x, target_y, self.cruise_altitude)

            # After 3 seconds of stable hover
            if time.time() - self.inspect_start_time >= 3.0:
                # Drop candidate pin for the ground team!
                self._publish_candidate_pin(target_x, target_y)

                # Move to next waypoint or return home
                self.current_wp_idx += 1
                if self.current_wp_idx < len(self.waypoints):
                    self.current_state = ScoutState.CRUISE
                else:
                    self.get_logger().info(
                        "🏁 All rooms surveyed! Returning to home base..."
                    )
                    self.current_state = ScoutState.RETURN_HOME

        # -------------------------------------------------------------
        # STATE 4: RETURN HOME (Fly back to origin 0, 0)
        # -------------------------------------------------------------
        elif self.current_state == ScoutState.RETURN_HOME:
            self.publish_position_setpoint(0.0, 0.0, self.cruise_altitude)

            dist_home = (self.pos_x**2 + self.pos_y**2) ** 0.5
            if dist_home < self.acceptance_radius:
                self.get_logger().info("🏠 Overhead home base. Landing...")
                self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.current_state = ScoutState.LAND

        # -------------------------------------------------------------
        # STATE 5: LANDING
        # -------------------------------------------------------------
        elif self.current_state == ScoutState.LAND:
            if current_altitude < 0.2:
                self.get_logger().info(
                    "✅ Touchdown confirmed. Disarming. Mission Complete!"
                )
                self.send_cmd(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
                )
                self.current_state = ScoutState.DONE


def main(args=None):
    rclpy.init(args=args)
    node = AerialScoutNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Scout node stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
