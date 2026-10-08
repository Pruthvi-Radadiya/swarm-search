from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
            ),
            Node(
                package="map_merge_swarm",
                executable="gt_map_tf_node",
                name="gt_map_tf_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="map_merge_swarm",
                executable="merge_node",
                name="map_merge_node",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="map_merge_swarm",
                executable="shared_map_relay",
                name="shared_map_relay",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
