"""Top-level bring-up for the team 6998 food cart.

Composes:
  - cart_description           (URDF -> robot_state_publisher)
  - sensors                    (lidar + D455, no D455 IMU)
  - nt_bridge                  (cmd_vel -> NT, NT -> /odom + /imu + TF)
  - slam_toolbox               (online async mapping; provides map -> odom)
  - nav2                       (planner + MPPI + behaviors + BT)
  - twist_mux                  (arbitrates nav / recovery / dock onto /cmd_vel)
  - dock_controller (optional) (AprilTag docking; disabled by default)
  - foxglove_bridge            (visualization)

/cmd_vel routing:
  nav2 controller   -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel_smooth ─┐
  nav2 behaviors    -> /cmd_vel_behavior ────────────────────────────────────┼─ twist_mux -> /cmd_vel -> nt_bridge
  dock_controller   -> /dock/cmd_vel ────────────────────────────────────────┘
                                       priorities: dock 100 > recovery 50 > nav 10

All sub-launches can be toggled off via launch args so we can isolate
problems during bring-up — e.g. enable_nav2:=false to test just SLAM.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
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
    default_map = PathJoinSubstitution([
        FindPackageShare('cart_bringup'), 'maps', 'floor_4', 'map'])
    nt_bridge_launch = PathJoinSubstitution([
        FindPackageShare('nt_bridge'), 'launch', 'nt_bridge.launch.py'])

    twist_mux_config = PathJoinSubstitution([
        FindPackageShare('cart_bringup'), 'config', 'twist_mux.yaml'])
    dock_config = PathJoinSubstitution([
        FindPackageShare('cart_elevator'), 'config', 'dock.yaml'])

    return LaunchDescription([
        # ---- toggles ----
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_description', default_value='true'),
        DeclareLaunchArgument('enable_sensors', default_value='true'),
        DeclareLaunchArgument('enable_camera', default_value='true',
                              description='Forward to sensors.launch; set false '
                              'to skip the D455 (not needed for mapping).'),
        DeclareLaunchArgument('enable_nt_bridge', default_value='true'),
        DeclareLaunchArgument('enable_slam', default_value='true'),
        DeclareLaunchArgument('slam_mode', default_value='mapping',
                              description="'mapping' (build a new map) or "
                              "'localization' (load map:= and localize). Use "
                              "localization to test Nav2 on a saved floor map."),
        DeclareLaunchArgument('map', default_value=default_map,
                              description='Localization only: serialized map '
                              'path WITHOUT extension. Defaults to floor_4; '
                              'point at maps/floor_N/map for other floors.'),
        DeclareLaunchArgument('enable_nav2', default_value='true'),
        DeclareLaunchArgument('enable_twist_mux', default_value='true'),
        DeclareLaunchArgument('enable_dock', default_value='false',
                              description='Launch dock_controller (off by default; '
                              'turn on when docking is desired).'),
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
            launch_arguments={
                'enable_camera': LaunchConfiguration('enable_camera'),
            }.items(),
        ),

        # ---- nt_bridge ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nt_bridge_launch),
            condition=IfCondition(LaunchConfiguration('enable_nt_bridge')),
        ),

        # ---- STARTUP STAGGER ----------------------------------------------
        # The Orange Pi 5 can't absorb ~19 nodes spawning at the same instant:
        # the concurrent plugin dlopen() + DDS discovery storm SIGSEGVs
        # bt_navigator (and SIGABRTs twist_mux), which silently leaves the
        # whole Nav2 lifecycle stuck 'inactive' -> costmaps/plans advertise
        # but never publish. So bring things up in dependency order with
        # TimerAction delays: foundation (description/sensors/nt_bridge) at
        # t=0 above, then SLAM, then the heavy Nav2 stack, then the rest.
        # Bump these periods up if a node still dies during bring-up.

        # ---- slam_toolbox (t=3s: needs /odom + /scan + TF flowing first) ---
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                condition=IfCondition(LaunchConfiguration('enable_slam')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'slam_mode': LaunchConfiguration('slam_mode'),
                    'map': LaunchConfiguration('map'),
                }.items(),
            ),
        ]),

        # ---- nav2 (t=7s: heaviest BT/plugin dlopen group; give it a clear
        #      window after the sensor+SLAM startup spike has settled) -------
        TimerAction(period=7.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                condition=IfCondition(LaunchConfiguration('enable_nav2')),
                launch_arguments={'use_sim_time': use_sim_time}.items(),
            ),
        ]),

        # ---- twist_mux + dock + foxglove (t=10s: after Nav2 is up) ---------
        TimerAction(period=10.0, actions=[
            # twist_mux arbitrates nav vs recovery vs dock onto /cmd_vel
            Node(
                package='twist_mux',
                executable='twist_mux',
                name='twist_mux',
                output='screen',
                parameters=[twist_mux_config, {'use_sim_time': use_sim_time}],
                remappings=[('cmd_vel_out', 'cmd_vel')],
                condition=IfCondition(LaunchConfiguration('enable_twist_mux')),
            ),
            # dock controller (optional)
            Node(
                package='cart_elevator',
                executable='dock_controller',
                name='dock_controller',
                output='screen',
                parameters=[dock_config, {'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('enable_dock')),
            ),
            # foxglove bridge (viz; subscribes broadly so keep it off the
            # critical Nav2 startup window)
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
        ]),
    ])
