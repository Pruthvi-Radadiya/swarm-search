import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sdf_path = os.path.expanduser("~/swarm-search/models/quadrotor/model.sdf")

    # Spawn quadrotor entity at (0.0, 0.0, 2.8)
    spawn_drone = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "quadrotor",
            "-file",
            sdf_path,
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "2.8",
            "-Y",
            "0.0",
        ],
        output="screen",
    )

    # Static TF for drone downward camera frame
    drone_cam_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="drone_cam_tf_publisher",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "-0.05",
            "--roll",
            "0.0",
            "--pitch",
            "1.570796",
            "--yaw",
            "0.0",
            "--frame-id",
            "drone/base_link",
            "--child-frame-id",
            "drone/camera_link",
        ],
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription(
        [
            spawn_drone,
            drone_cam_tf,
        ]
    )
