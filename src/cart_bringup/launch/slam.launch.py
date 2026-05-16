"""Run slam_toolbox in online async mapping mode.

Requires:
  - /scan from the lidar driver
  - odom -> base_link TF (published by nt_bridge once RoboRIO is up;
    sits at identity otherwise, which is OK for stationary bring-up)
  - base_link TF tree from cart_description / robot_state_publisher
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params = LaunchConfiguration('slam_params_file')

    default_params = PathJoinSubstitution([
        FindPackageShare('cart_bringup'), 'config', 'slam_toolbox.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'slam_params_file', default_value=default_params,
            description='Path to slam_toolbox YAML'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
        ),
    ])
