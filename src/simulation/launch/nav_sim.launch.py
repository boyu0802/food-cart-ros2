"""
Launch file for Navigation stack with SLAM and cart_elevator mission.
Uses the real config files from cart_bringup and cart_elevator packages.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    map_arg = DeclareLaunchArgument(
        'map', default_value='',
        description='Map file (empty for SLAM mode)'
    )
    slam_arg = DeclareLaunchArgument(
        'slam', default_value='true',
        description='Run SLAM (true) or localization (false)'
    )

    # Package share paths
    cart_bringup_share = FindPackageShare('cart_bringup')
    cart_elevator_share = FindPackageShare('cart_elevator')

    # Config files
    nav2_config = PathJoinSubstitution([
        cart_bringup_share, 'config', 'nav2.yaml'
    ])
    slam_config = PathJoinSubstitution([
        cart_bringup_share, 'config', 'slam_toolbox.yaml'
    ])
    twist_mux_config = PathJoinSubstitution([
        cart_bringup_share, 'config', 'twist_mux.yaml'
    ])
    scan_filter_config = PathJoinSubstitution([
        cart_bringup_share, 'config', 'scan_filter.yaml'
    ])

    cart_elevator_config = PathJoinSubstitution([
        cart_elevator_share, 'config', 'mission.yaml'
    ])
    safe_config = PathJoinSubstitution([
        cart_elevator_share, 'config', 'safe.yaml'
    ])
    dock_config = PathJoinSubstitution([
        cart_elevator_share, 'config', 'dock.yaml'
    ])
    door_hallway_config = PathJoinSubstitution([
        cart_elevator_share, 'config', 'door_hallway.yaml'
    ])
    door_incab_config = PathJoinSubstitution([
        cart_elevator_share, 'config', 'door_incab.yaml'
    ])
    floor_config = PathJoinSubstitution([
        cart_elevator_share, 'config', 'floor.yaml'
    ])

    # =============================
    # Scan filter (masks cart structure from lidar)
    # =============================
    scan_filter = Node(
        package='cart_bringup',
        executable='scan_sector_filter',
        name='scan_sector_filter',
        output='screen',
        parameters=[scan_filter_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/scan_raw', '/scan_raw'),
            ('/scan', '/scan'),
        ]
    )

    # =============================
    # Twist mux for command arbitration
    # =============================
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/cmd_vel_out', '/cmd_vel'),
        ]
    )

    # =============================
    # Nav2 Bringup
    # =============================
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav2_config,
            'map': LaunchConfiguration('map'),
            'slam': LaunchConfiguration('slam'),
        }.items()
    )

    # =============================
    # SLAM Toolbox (if slam=true)
    # =============================
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/scan', '/scan'),
            ('/map', '/map'),
        ],
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    # =============================
    # Cart Elevator Mission Stack
    # =============================

    # Supervisor - the 18-state FSM
    supervisor = Node(
        package='cart_elevator',
        executable='cart_supervisor',
        name='supervisor',
        output='screen',
        parameters=[
            cart_elevator_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/elevator/call', '/elevator/call'),
            ('/elevator/goto_floor', '/elevator/goto_floor'),
            ('/elevator/door_state', '/elevator/door_state'),
            ('/elevator/direction', '/elevator/direction'),
            ('/elevator/floor', '/elevator/floor'),
            ('/elevator/safe_to_enter', '/elevator/safe_to_enter'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    # Safe-to-enter gate
    safe_gate = Node(
        package='cart_elevator',
        executable='safe_to_enter_gate',
        name='safe_to_enter_gate',
        output='screen',
        parameters=[
            safe_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/elevator/door_state', '/elevator/door_state'),
            ('/elevator/direction', '/elevator/direction'),
            ('/elevator/floor', '/elevator/floor'),
            ('/elevator/inside_clear', '/elevator/inside_clear'),
            ('/elevator/safe_to_enter', '/elevator/safe_to_enter'),
        ]
    )

    # Hallway door detector (waiting for elevator door to open)
    door_hallway = Node(
        package='cart_elevator',
        executable='door_state_detector',
        name='door_state_detector_hallway',
        output='screen',
        parameters=[
            door_hallway_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/camera/camera/depth/image_rect_raw', '/camera/depth/image'),
            ('/elevator/door_state', '/elevator/door_state'),
        ]
    )

    # In-cab door detector (inside elevator cab)
    door_incab = Node(
        package='cart_elevator',
        executable='door_state_detector',
        name='door_state_detector_incab',
        output='screen',
        parameters=[
            door_incab_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/camera/camera/depth/image_rect_raw', '/camera/depth/image'),
            ('/elevator/door_state', '/elevator/door_state_incab'),
        ]
    )

    # Floor detector stack
    floor_v1 = Node(
        package='cart_elevator',
        executable='floor_v1_apriltag',
        name='floor_v1_apriltag',
        output='screen',
        parameters=[
            floor_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/limelight/tag', '/limelight/tag'),
            ('/elevator/floor_v1_apriltag', '/elevator/floor_v1_apriltag'),
        ]
    )

    floor_v2 = Node(
        package='cart_elevator',
        executable='floor_v2_time',
        name='floor_v2_time',
        output='screen',
        parameters=[
            floor_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/imu', '/imu_raw'),
            ('/elevator/floor_v2_time', '/elevator/floor_v2_time'),
        ]
    )

    floor_v3 = Node(
        package='cart_elevator',
        executable='floor_v3_accel',
        name='floor_v3_accel',
        output='screen',
        parameters=[
            floor_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/imu', '/imu_raw'),
            ('/elevator/floor_v3_accel', '/elevator/floor_v3_accel'),
        ]
    )

    floor_v4 = Node(
        package='cart_elevator',
        executable='floor_v4_fusion',
        name='floor_v4_fusion',
        output='screen',
        parameters=[
            floor_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/elevator/floor_v1_apriltag', '/elevator/floor_v1_apriltag'),
            ('/elevator/floor_v2_time', '/elevator/floor_v2_time'),
            ('/elevator/floor_v3_accel', '/elevator/floor_v3_accel'),
            ('/elevator/floor', '/elevator/floor'),
        ]
    )

    # Direction detector stack
    direction_v1 = Node(
        package='cart_elevator',
        executable='direction_v1_digit',
        name='direction_v1_digit',
        output='screen',
        parameters=[
            floor_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/image', '/image'),
            ('/elevator/direction_v1_digit', '/elevator/direction_v1_digit'),
        ]
    )

    direction_v3 = Node(
        package='cart_elevator',
        executable='direction_v3_fusion',
        name='direction_v3_fusion',
        output='screen',
        parameters=[
            floor_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/elevator/direction_v1_digit', '/elevator/direction_v1_digit'),
            ('/elevator/direction', '/elevator/direction'),
        ]
    )

    # Dock controller
    dock_control = Node(
        package='cart_elevator',
        executable='dock_controller',
        name='dock_control',
        output='screen',
        parameters=[
            dock_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    # =============================
    # RViz (optional)
    # =============================
    rviz_config = PathJoinSubstitution([
        cart_bringup_share, 'rviz', 'nav2.rviz'
    ])
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2'
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_arg,
        slam_arg,
        rviz_arg,
        # Core perception
        scan_filter,
        twist_mux,
        # Nav2 + SLAM
        nav2_bringup,
        slam_toolbox,
        # Cart elevator stack
        supervisor,
        safe_gate,
        door_hallway,
        door_incab,
        floor_v1,
        floor_v2,
        floor_v3,
        floor_v4,
        direction_v1,
        direction_v3,
        dock_control,
        # Visualization
        rviz,
    ])