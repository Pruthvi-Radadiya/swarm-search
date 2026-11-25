#!/usr/bin/env python3
"""
unified_swarm_house.launch.py - Master Unified Air-Ground Swarm Simulation Launch.

Spawns in Modern Gazebo Sim:
1. World: turtlebot3_house.world
2. 3x Anti-Tip Flat Mini-Rovers: TB3_1, TB3_2, TB3_3 with 2D Lidar + Forward RGB Cam
3. 1x Aerial PX4 Quadrotor: x500_downward_cam with Downward RGB Cam
4. Full ros_gz_bridge wiring for all 4 camera streams, scans, and velocity commands.
"""

import os

from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    workspace_dir = os.path.expanduser("~/swarm-search")
    world_path = os.path.join(workspace_dir, "worlds", "house_garden.sdf")
    mini_rover_sdf = os.path.join(workspace_dir, "models", "mini_rover", "model.sdf")

    x500_sdf = os.path.join(workspace_dir, "models", "x500_downward_cam", "model.sdf")

    gz_resource_path = (
        f"{os.path.expanduser('~/swarm-search/models')}:"
        f"{os.path.expanduser('~/.gazebo/models')}:"
        f"{os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models')}:"
        f"{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"
    )

    # 1. Start Modern Gazebo Sim with House World and Furniture Paths
    gz_sim = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_path],
        output="screen",
        additional_env={
            "GZ_SIM_RESOURCE_PATH": gz_resource_path,
            "LIBGL_ALWAYS_SOFTWARE": "1",
        },
    )

    # 2. Spawning 3 Flat Mini-Rovers (TB3_1, TB3_2, TB3_3)
    # TB3_1: In front of house, clear of Postbox and Wall (x=0.5, y=-1.5)
    spawn_tb1 = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            "/world/house_garden/create",
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            f'sdf_filename: "{mini_rover_sdf}", name: "TB3_1", pose: {{position: {{x: 0.5, y: -1.5, z: 0.05}}}}',
        ],
        output="screen",
    )

    # TB3_2: Living Room / Central Hall (x=-0.3, y=0.4)
    spawn_tb2 = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            "/world/house_garden/create",
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            f'sdf_filename: "{mini_rover_sdf}", name: "TB3_2", pose: {{position: {{x: -0.3, y: 0.4, z: 0.05}}}}',
        ],
        output="screen",
    )

    # TB3_3: Inside Room near Dustbin / Trash Can (x=1.5, y=1.6)
    spawn_tb3 = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            "/world/house_garden/create",
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            f'sdf_filename: "{mini_rover_sdf}", name: "TB3_3", pose: {{position: {{x: 1.5, y: 1.6, z: 0.05}}}}',
        ],
        output="screen",
    )

    # 4. Spawning Aerial PX4 Drone (x500_downward_cam)
    spawn_drone = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            "/world/house_garden/create",
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            f'sdf_filename: "{x500_sdf}", name: "x500_downward_cam", pose: {{position: {{x: 0.5, y: -3.2, z: 0.2}}}}',
        ],
        output="screen",
    )

    # 3. Master Multi-Robot ROS <-> Gazebo Bridge
    bridge_args = [
        # Aerial Drone Downward Camera
        "/camera@sensor_msgs/msg/Image[gz.msgs.Image",
        "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        # Rover 1
        "/model/TB3_1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/model/TB3_1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        # Rover 2
        "/model/TB3_2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/model/TB3_2/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        # Rover 3
        "/model/TB3_3/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "/model/TB3_3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    ]

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="unified_swarm_bridge",
        output="screen",
        arguments=bridge_args,
        remappings=[
            ("/camera", "/drone/camera/image_raw"),
            ("/camera_info", "/drone/camera/camera_info"),
            ("/model/TB3_1/cmd_vel", "/TB3_1/cmd_vel"),
            ("/model/TB3_1/odometry", "/TB3_1/odom"),
            ("/model/TB3_2/cmd_vel", "/TB3_2/cmd_vel"),
            ("/model/TB3_2/odometry", "/TB3_2/odom"),
            ("/model/TB3_3/cmd_vel", "/TB3_3/cmd_vel"),
            ("/model/TB3_3/odometry", "/TB3_3/odom"),
        ],
    )

    return LaunchDescription(
        [
            gz_sim,
            spawn_tb1,
            spawn_tb2,
            spawn_tb3,
            spawn_drone,
            gz_bridge_node,
        ]
    )
