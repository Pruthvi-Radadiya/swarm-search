#!/usr/bin/env python3
"""
GTA-V Style PX4 Flight Controller using Pygame (SDL2).
Direct hardware multi-key input: Hold 8+4 (diagonal), W+8 (climb+forward), etc.
"""

import math
import threading
import time

import pygame
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)


class PX4Teleop(Node):
    def __init__(self):
        super().__init__("px4_teleop")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )
        self.odom_sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._odom_callback, qos
        )

        # Drone State
        self.yaw = 0.0
        self.altitude = 0.0
        self.is_taking_off = False
        self.status_msg = "Ready to fly"

        # Speeds
        self.lin_speed = 1.5  # m/s
        self.vert_speed = 0.8  # m/s
        self.yaw_speed = 1.0  # rad/s

        self.vx_body = 0.0
        self.vy_body = 0.0
        self.vz = 0.0
        self.wz = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = -2.5
        self.hold_x = None
        self.hold_y = None
        self.hold_z = None

        # Heart-beat timer (20 Hz)
        self.timer = self.create_timer(0.05, self._timer_callback)

    def _odom_callback(self, msg: VehicleOdometry):
        self.altitude = -msg.position[2]
        self.current_x = msg.position[0]
        self.current_y = msg.position[1]
        self.current_z = msg.position[2]

        if self.is_taking_off and self.altitude >= 2.5:
            self.vz = 0.0
            self.is_taking_off = False
            self.hold_z = self.current_z  # <-- ADD THIS LINE
            self.status_msg = "Holding at 2.5m"

        q = msg.q
        siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def _timer_callback(self):
        # 1. Enable Position & Velocity in Offboard Mode
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(offboard_msg)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # -------------------------------------------------------------
        # 2. HORIZONTAL AXIS (Pitch & Roll: 8, 5, 4, 6)
        # -------------------------------------------------------------
        if abs(self.vx_body) > 0.01 or abs(self.vy_body) > 0.01:
            # Active steering: fly with velocity
            self.hold_x = None
            self.hold_y = None
            cos_yaw = math.cos(self.yaw)
            sin_yaw = math.sin(self.yaw)
            pos_x = float("nan")
            pos_y = float("nan")
            vel_x = float(self.vx_body * cos_yaw - self.vy_body * sin_yaw)
            vel_y = float(self.vx_body * sin_yaw + self.vy_body * cos_yaw)
        else:
            # Keys released: LOCK HORIZONTAL POSITION IMMEDIATELY!
            if self.hold_x is None:
                self.hold_x = self.current_x
                self.hold_y = self.current_y
            pos_x = float(self.hold_x)
            pos_y = float(self.hold_y)
            vel_x = 0.0
            vel_y = 0.0

        # -------------------------------------------------------------
        # 3. VERTICAL AXIS (Altitude: W, S)
        # -------------------------------------------------------------
        if self.is_taking_off or abs(self.vz) > 0.01:
            # Active climbing or descending
            self.hold_z = None
            pos_z = float("nan")
            vel_z = float(self.vz)
        else:
            # W/S released: LOCK ALTITUDE IMMEDIATELY!
            if self.hold_z is None:
                self.hold_z = self.current_z
            pos_z = float(self.hold_z)
            vel_z = 0.0

        # -------------------------------------------------------------
        # 4. YAW AXIS (Turn: A, D)
        # -------------------------------------------------------------
        yaw_rate = float(self.wz) if abs(self.wz) > 0.01 else 0.0

        # Publish the setpoint
        setpoint.position = [pos_x, pos_y, pos_z]
        setpoint.velocity = [vel_x, vel_y, vel_z]
        setpoint.yaw = float("nan")
        setpoint.yawspeed = yaw_rate
        self.trajectory_setpoint_pub.publish(setpoint)

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
        self.vehicle_command_pub.publish(cmd)

    def takeoff(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        time.sleep(0.1)
        self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.is_taking_off = True
        self.vz = -self.vert_speed
        self.status_msg = "Auto-climbing to 2.5m..."

    def land(self):
        self.is_taking_off = False
        self.hover_stop()
        self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.status_msg = "Landing..."

    def hover_stop(self):
        self.is_taking_off = False
        self.vx_body = 0.0
        self.vy_body = 0.0
        self.vz = 0.0
        self.wz = 0.0
        self.status_msg = "Hovering"


def main():
    rclpy.init()
    node = PX4Teleop()

    # Spin ROS 2 in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Initialize Pygame Display (Game HUD)
    pygame.init()
    screen_width, screen_height = 540, 360
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("🎮 PX4 GTA-V Flight Controller (Direct Hardware)")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("monospace", 16)
    title_font = pygame.font.SysFont("monospace", 20, bold=True)

    running = True
    while running and rclpy.ok():
        # Handle Instant Events (Takeoff / Land / Speed / Exit)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_t:
                    node.takeoff()
                elif event.key == pygame.K_l:
                    node.land()
                elif event.key == pygame.K_SPACE:
                    node.hover_stop()
                elif event.key in [pygame.K_EQUALS, pygame.K_PLUS]:
                    node.lin_speed = min(5.0, node.lin_speed + 0.2)
                elif event.key in [pygame.K_MINUS, pygame.K_UNDERSCORE]:
                    node.lin_speed = max(0.2, node.lin_speed - 0.2)

        # -------------------------------------------------------------
        # Direct Hardware Key State (GTA 5 Style Multi-Key Combining)
        # -------------------------------------------------------------
        keys = pygame.key.get_pressed()

        # 1. Pitch (Forward / Back)
        vx = 0.0
        if keys[pygame.K_8] or keys[pygame.K_KP8] or keys[pygame.K_UP]:
            vx += node.lin_speed
        if keys[pygame.K_5] or keys[pygame.K_KP5] or keys[pygame.K_DOWN]:
            vx -= node.lin_speed
        node.vx_body = vx

        # 2. Roll (Strafe Left / Right)
        vy = 0.0
        if keys[pygame.K_4] or keys[pygame.K_KP4] or keys[pygame.K_LEFT]:
            vy -= node.lin_speed
        if keys[pygame.K_6] or keys[pygame.K_KP6] or keys[pygame.K_RIGHT]:
            vy += node.lin_speed
        node.vy_body = vy

        # 3. Altitude (Climb / Descend)
        if not node.is_taking_off:
            vz = 0.0
            if keys[pygame.K_w]:
                vz -= node.vert_speed  # W: climb
            if keys[pygame.K_s]:
                vz += node.vert_speed  # S: descend
            node.vz = vz

        # 4. Heading / Yaw (Turn Left / Right)
        wz = 0.0
        if keys[pygame.K_a]:
            wz -= node.yaw_speed  # A: turn left
        if keys[pygame.K_d]:
            wz += node.yaw_speed  # D: turn right
        node.wz = wz

        # -------------------------------------------------------------
        # Draw Real-Time Game HUD Window
        # -------------------------------------------------------------
        screen.fill((20, 24, 32))  # Dark sleek background

        # Title
        title_surf = title_font.render("🎮 GTA-V DRONE CONTROLLER", True, (0, 255, 180))
        screen.blit(title_surf, (20, 15))

        # Status & Telemetry
        telemetry = [
            f"Status : {node.status_msg}",
            f"Alt    : {node.altitude:.2f} m",
            f"Pitch  : {node.vx_body:+.1f} m/s  (Forward/Back)",
            f"Roll   : {node.vy_body:+.1f} m/s  (Strafe Left/Right)",
            f"Climb  : {-node.vz:+.1f} m/s  (Up/Down)",
            f"Turn   : {node.wz:+.1f} rad/s",
            f"Speed  : {node.lin_speed:.1f} m/s",
        ]
        y_offset = 55
        for line in telemetry:
            t_surf = font.render(line, True, (220, 225, 235))
            screen.blit(t_surf, (25, y_offset))
            y_offset += 24

        # Key Legend
        pygame.draw.line(screen, (50, 60, 80), (20, 235), (screen_width - 20, 235), 1)
        legend = [
            "Controls: [W/S] Climb/Descend  |  [A/D] Turn Yaw",
            "          [8/5] Pitch Fwd/Back |  [4/6] Roll Left/Right",
            "Hold 8+4 together -> Flies Diagonally!",
            "[T] Takeoff 2.5m | [L] Land | [Space] Brake | [Esc] Exit",
        ]
        y_leg = 245
        for leg in legend:
            leg_surf = font.render(leg, True, (130, 150, 180))
            screen.blit(leg_surf, (20, y_leg))
            y_leg += 22

        pygame.display.flip()
        clock.tick(50)  # 50 Hz game loop

    # Cleanup
    node.hover_stop()
    pygame.quit()
    node.destroy_node()
    rclpy.shutdown()
    print("\nController closed cleanly.")


if __name__ == "__main__":
    main()
