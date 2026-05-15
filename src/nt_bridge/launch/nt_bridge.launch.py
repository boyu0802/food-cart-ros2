from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('nt_bridge'),
        'config',
        'nt_bridge.yaml',
    )

    return LaunchDescription([
        Node(
            package='nt_bridge',
            executable='nt_bridge_node',
            name='nt_bridge',
            output='screen',
            parameters=[config],
        ),
    ])
