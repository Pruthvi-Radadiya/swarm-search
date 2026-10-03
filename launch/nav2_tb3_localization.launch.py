import os

from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node

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

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace=robot_ns,
        output="screen",
        # Keep TF on global /tf (not /TB3_2/tf) so controller/planner see shared_map.
        remappings=[("/tf", "/tf"), ("/tf_static", "/tf_static")],
        parameters=[
            params_file,
            {
                "use_sim_time": True,
                "base_frame_id": "TB3_2/base_footprint",
                "odom_frame_id": "TB3_2/odom",
                "global_frame_id": "shared_map",
                "scan_topic": "scan",
                "map_topic": "/map",
                "tf_broadcast": True,
            },
        ],
    )

    amcl_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"node_names": ["amcl"]},
        ],
        namespace=robot_ns,
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ns", default_value="TB3_2"),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.expanduser(
                    "~/swarm-search/nav2_params/burger_tb3_2_loc.yaml"
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            nav2_group,
            amcl_node,
            amcl_lifecycle,
        ]
    )
