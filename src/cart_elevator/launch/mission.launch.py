"""Integrated mission layer for a REAL run (not the sim).

Brings up the mission brain plus the elevator perception / docking nodes
that build the Snapshot it reacts to. Assumes the navigation foundation
(SLAM/localization, Nav2, sensors, nt_bridge, twist_mux) is ALREADY up
via cart_bringup `bringup.launch.py` — run that first, then this. Keeping
them separate avoids the spawn-storm that SIGSEGVs Nav2 on the Orange Pi
(see the stagger note in bringup.launch.py).

The elevator-sense, dock, and safe-gate groups are gated behind launch
args and default OFF, because their real inputs aren't wired/calibrated
yet:
  - direction detectors need a real /image of the in-cab display,
  - door_state_detector needs a hand-picked depth ROI per door,
  - the safe gate ANDs both of those, so it can't open until they work.
Turn each group on as it matures. The supervisor itself always launches.

Ride-only / cab-exit bring-up (today's first test):
  Configure mission.yaml with first_state + after_cab_exit (e.g.
  NAV_OUT_OF_CAB -> DONE), leave the groups off, hold the dead-man, and
  publish /elevator/safe_to_enter by hand if a WAIT state is in play.

    ros2 launch cart_bringup bringup.launch.py \\
        slam_mode:=localization map:=<ws>/maps/floor_4/map
    ros2 launch cart_elevator mission.launch.py

Full ride later:
    ros2 launch cart_elevator mission.launch.py \\
        enable_docks:=true enable_elevator_sense:=true enable_safe_gate:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('cart_elevator')
    mission_yaml = PathJoinSubstitution([pkg, 'config', 'mission.yaml'])
    floor_yaml = PathJoinSubstitution([pkg, 'config', 'floor.yaml'])
    dock_yaml = PathJoinSubstitution([pkg, 'config', 'dock.yaml'])
    safe_yaml = PathJoinSubstitution([pkg, 'config', 'safe.yaml'])
    # Door ROI/thresholds differ per dock pose; pick the file via door_config
    # (door_hallway.yaml for Test A, door_incab.yaml for Test B).
    door_yaml = PathJoinSubstitution([pkg, 'config',
                                      LaunchConfiguration('door_config')])

    use_sim_time = LaunchConfiguration('use_sim_time')
    sense = IfCondition(LaunchConfiguration('enable_elevator_sense'))
    docks = IfCondition(LaunchConfiguration('enable_docks'))
    safe = IfCondition(LaunchConfiguration('enable_safe_gate'))
    door = IfCondition(LaunchConfiguration('enable_door'))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        # Default OFF: real inputs for these aren't wired/calibrated yet.
        DeclareLaunchArgument('enable_elevator_sense', default_value='false',
                              description='floor_v4 + direction stack '
                              '(needs real /image of the in-cab display).'),
        DeclareLaunchArgument('enable_door', default_value='false',
                              description='door_state_detector alone (needs a '
                              'hand-picked D455 depth ROI per dock pose).'),
        DeclareLaunchArgument('door_config', default_value='door_hallway.yaml',
                              description='Which door param file to load: '
                              'door_hallway.yaml (Test A) or door_incab.yaml '
                              '(Test B). ROI differs per pose.'),
        DeclareLaunchArgument('enable_docks', default_value='false',
                              description='AprilTag dock + in-cab lidar wall dock.'),
        DeclareLaunchArgument('enable_safe_gate', default_value='false',
                              description='safe_to_enter_gate. With '
                              'safe_require_direction/floor:=false it gates on '
                              'DOOR==OPEN alone (bring-up door-only mode).'),
        DeclareLaunchArgument('safe_require_direction', default_value='true',
                              description='Set false to drop the direction==IDLE '
                              'precondition (placeholder detector).'),
        DeclareLaunchArgument('safe_require_floor', default_value='true',
                              description='Set false to drop the floor==target '
                              'precondition.'),

        # ---- mission brain (always on) ----
        # require_enable defaults True in the node — the operator holds the
        # RoboRIO dead-man (/mission/enable via nt_bridge) on real runs.
        Node(package='cart_elevator', executable='cart_supervisor',
             name='cart_supervisor', output='screen',
             parameters=[mission_yaml, {'use_sim_time': use_sim_time}]),

        # ---- safe gate (optional) ----
        Node(package='cart_elevator', executable='safe_to_enter_gate',
             name='safe_to_enter_gate', output='screen',
             parameters=[safe_yaml, {
                 'use_sim_time': use_sim_time,
                 'require_direction': LaunchConfiguration('safe_require_direction'),
                 'require_floor': LaunchConfiguration('safe_require_floor'),
             }],
             condition=safe),

        # ---- docking (optional) ----
        Node(package='cart_elevator', executable='dock_controller',
             name='dock_controller', output='screen',
             parameters=[dock_yaml, {'use_sim_time': use_sim_time}],
             condition=docks),
        Node(package='cart_elevator', executable='wall_dock_controller',
             name='wall_dock_controller', output='screen',
             # No dedicated yaml yet; in-cab geometry from wall_dock_test.
             # Tune side_target/standoff_x against the real cab.
             parameters=[{'standoff_x': 0.30, 'side': 'left',
                          'side_target': 0.70, 'autostart': False,
                          'use_sim_time': use_sim_time}],
             condition=docks),

        # ---- elevator sensing (optional) ----
        Node(package='cart_elevator', executable='floor_v4_fusion',
             name='floor_v4_fusion', output='screen',
             parameters=[floor_yaml, {'use_sim_time': use_sim_time}],
             condition=sense),
        Node(package='cart_elevator', executable='floor_v1_apriltag',
             name='floor_v1_apriltag', output='screen',
             parameters=[floor_yaml, {'use_sim_time': use_sim_time}],
             condition=sense),
        Node(package='cart_elevator', executable='floor_v3_accel',
             name='floor_v3_accel', output='screen',
             parameters=[floor_yaml, {'use_sim_time': use_sim_time}],
             condition=sense),
        Node(package='cart_elevator', executable='direction_v1_digit',
             name='direction_v1_digit', output='screen',
             parameters=[floor_yaml, {'use_sim_time': use_sim_time}],
             condition=sense),
        Node(package='cart_elevator', executable='direction_v2_flow',
             name='direction_v2_flow', output='screen',
             parameters=[floor_yaml, {'use_sim_time': use_sim_time}],
             condition=sense),
        Node(package='cart_elevator', executable='direction_v3_fusion',
             name='direction_v3_fusion', output='screen',
             parameters=[floor_yaml, {'use_sim_time': use_sim_time}],
             condition=sense),
        # ---- door detector (its own toggle; ROI must be calibrated per
        #      dock pose with rqt_image_view) ----
        Node(package='cart_elevator', executable='door_state_detector',
             name='door_state_detector', output='screen',
             parameters=[door_yaml, {'use_sim_time': use_sim_time}],
             condition=door),
    ])
