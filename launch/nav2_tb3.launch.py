import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace

from launch import LaunchDescription


def generate_launch_description():
    robot_ns = LaunchConfiguration("robot_ns")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    nav2_launch = os.path.join(
    os.path.expanduser("~/swarm-search/launch"), "navigation_launch.py"
)

    nav2_group = GroupAction(
        [
            PushRosNamespace(robot_ns),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "namespace": robot_ns,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": "true",
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ns", default_value="TB3_1"),
            DeclareLaunchArgument("params_file"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            nav2_group,
        ]
    )
