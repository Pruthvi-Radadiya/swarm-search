from launch import LaunchDescription
from launch_ros.actions import Node

from map_merge_swarm.spawn_offsets import SPAWN_OFFSETS

def generate_launch_description():
    nodes = []
    for robot_ns, (x, y, yaw) in SPAWN_OFFSETS.items():
        nodes.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_{robot_ns.lower()}',
                arguments=[
                    '--x', str(x),
                    '--y', str(y),
                    '--z', '0',
                    '--yaw', str(yaw),
                    '--pitch', '0',
                    '--roll', '0',
                    '--frame-id', 'shared_map',
                    '--child-frame-id', f'{robot_ns}/map',
                ])
        )
    return LaunchDescription(nodes)

