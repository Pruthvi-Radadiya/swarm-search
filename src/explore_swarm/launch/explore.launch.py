from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="explore_swarm",
                executable="frontier_detector",
                name="frontier_detector",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="explore_swarm",
                executable="goal_assigner",
                name="goal_assigner",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
