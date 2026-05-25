"""Run slam_toolbox in online async mapping mode.

Requires:
  - /scan from the lidar driver (via scan_sector_filter)
  - odom -> base_footprint TF (published by nt_bridge once RoboRIO is up)
  - base_footprint -> base_link -> laser TF from cart_description / RSP

On Jazzy, async_slam_toolbox_node is a LIFECYCLE node, so a
nav2_lifecycle_manager (autostart) configures+activates it. Without that it
sits in the unconfigured state and never creates /scan or /map.
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

        # async_slam_toolbox_node is a lifecycle node on Jazzy; this manager
        # configures+activates it on startup (autostart). Without it, slam
        # never creates /scan or /map.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_slam',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['slam_toolbox'],
            }],
        ),
    ])
