"""Blind plumbing test for the arrow-based fallback detector.

Launches the synthetic marching-block image source against direction_v2_flow
+ direction_v3_fusion. The synthetic source has no digits, so direction_v1_digit
is NOT included here -- use direction_video_test.launch.py for that.

NOTE: direction_v2_flow is still a PLACEHOLDER and emits UNKNOWN/0.0 until
the optical-flow algorithm is filled in. This launch is for verifying the
message plumbing and the fusion logic.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('cart_elevator'), 'config', 'floor.yaml'])

    scenario = LaunchConfiguration('scenario')

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='+2,p4,-2'),

        Node(package='cart_elevator', executable='fake_direction_image',
             name='fake_direction_image', output='screen',
             parameters=[params_file, {'scenario': scenario}]),

        Node(package='cart_elevator', executable='direction_v2_flow',
             name='direction_v2_flow', output='screen',
             parameters=[params_file]),
        Node(package='cart_elevator', executable='direction_v3_fusion',
             name='direction_v3_fusion', output='screen',
             parameters=[params_file]),
    ])
