#!/usr/bin/env python3
"""
Launch file for bridging Gazebo Garden topics to ROS 2 for the PX4 drone.
Bridges:
- /camera (gz.msgs.Image) -> /drone/camera/image_raw (sensor_msgs/msg/Image)
- /camera_info (gz.msgs.CameraInfo) -> /drone/camera/camera_info (sensor_msgs/msg/CameraInfo)
"""

from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    # Bridge definition: <gz_topic>@<ros_type>[<gz_type>
    camera_image_bridge = "/camera@sensor_msgs/msg/Image[gz.msgs.Image"
    camera_info_bridge = "/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="drone_gz_bridge",
        output="screen",
        arguments=[
            camera_image_bridge,
            camera_info_bridge,
        ],
        remappings=[
            ("/camera", "/drone/camera/image_raw"),
            ("/camera_info", "/drone/camera/camera_info"),
        ],
    )

    return LaunchDescription(
        [
            gz_bridge_node,
        ]
    )
