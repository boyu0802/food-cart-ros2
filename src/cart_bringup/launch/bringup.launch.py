"""Top-level bring-up for the team 6998 food cart.

Composes:
  - cart_description           (URDF -> robot_state_publisher)
  - sensors                    (lidar + D455, no D455 IMU)
  - nt_bridge                  (cmd_vel -> NT, NT -> /odom + /imu + TF)
  - slam_toolbox               (online async mapping; provides map -> odom)
  - nav2                       (planner + MPPI + behaviors + BT)
  - foxglove_bridge            (visualization)

All sub-launches can be toggled off via launch args so we can isolate
problems during bring-up — e.g. enable_nav2:=false to test just SLAM.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    desc_launch = PathJoinSubstitution([
        FindPackageShare('cart_description'), 'launch', 'description.launch.py'])
    sensors_launch = PathJoinSubstitution([
        FindPackageShare('cart_bringup'), 'launch', 'sensors.launch.py'])
    slam_launch = PathJoinSubstitution([
        FindPackageShare('cart_bringup'), 'launch', 'slam.launch.py'])
    nav2_launch = PathJoinSubstitution([
        FindPackageShare('cart_bringup'), 'launch', 'nav2.launch.py'])
    nt_bridge_launch = PathJoinSubstitution([
        FindPackageShare('nt_bridge'), 'launch', 'nt_bridge.launch.py'])

    return LaunchDescription([
        # ---- toggles ----
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_description', default_value='true'),
        DeclareLaunchArgument('enable_sensors', default_value='true'),
        DeclareLaunchArgument('enable_nt_bridge', default_value='true'),
        DeclareLaunchArgument('enable_slam', default_value='true'),
        DeclareLaunchArgument('enable_nav2', default_value='true'),
        DeclareLaunchArgument('enable_foxglove', default_value='true'),

        # ---- description ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(desc_launch),
            condition=IfCondition(LaunchConfiguration('enable_description')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # ---- sensors ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensors_launch),
            condition=IfCondition(LaunchConfiguration('enable_sensors')),
        ),

        # ---- nt_bridge ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nt_bridge_launch),
            condition=IfCondition(LaunchConfiguration('enable_nt_bridge')),
        ),

        # ---- slam_toolbox ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            condition=IfCondition(LaunchConfiguration('enable_slam')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # ---- nav2 ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            condition=IfCondition(LaunchConfiguration('enable_nav2')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # ---- foxglove bridge ----
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'use_sim_time': use_sim_time,
            }],
            condition=IfCondition(LaunchConfiguration('enable_foxglove')),
        ),
    ])
