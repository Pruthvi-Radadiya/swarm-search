import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, SetRemap

from launch import LaunchDescription


def generate_launch_description():
    robot_ns = LaunchConfiguration("robot_ns")
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    slam_toolbox_launch = os.path.join(
        get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py"
    )

    slam_group = GroupAction(
        [
            PushRosNamespace(robot_ns),
            SetRemap(src="/map", dst="map"),
            SetRemap(src="/map_metadata", dst="map_metadata"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "slam_params_file": slam_params_file,
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ns", default_value="TB3_1"),
            DeclareLaunchArgument("slam_params_file"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            slam_group,
        ]
    )
