"""
Launch file for Gazebo simulation with school world and food cart robot.
Starts Gazebo, spawns the robot, ros_gz_bridge, and foxglove_bridge.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value='school.sdf',
        description='Gazebo world file'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (no GUI)'
    )

    # Paths
    sim_dir = PathJoinSubstitution([
        FindPackageShare('simulation'), '..', '..', 'simulation'
    ])
    world_path = PathJoinSubstitution([
        sim_dir, 'worlds', LaunchConfiguration('world')
    ])
    robot_model_path = PathJoinSubstitution([
        sim_dir, 'models', 'food_cart', 'model.sdf'
    ])
    bridge_config_path = PathJoinSubstitution([
        sim_dir, 'config', 'gz_bridge.yaml'
    ])

    # Gazebo server
    gz_server = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-s', '-r',
            world_path
        ],
        output='screen',
        name='gz_sim_server'
    )

    # Gazebo client (GUI) - optional (skip when headless=true)
    gz_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        name='gz_sim_client',
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )

    # Spawn robot model
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-file', robot_model_path,
            '-name', 'food_cart',
            '-x', '0', '-y', '0', '-z', '0.1',
            '-Y', '0'
        ],
        output='screen',
        name='spawn_robot'
    )

    # ros_gz_bridge - bridges Gazebo topics to ROS2
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config_path,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Static TF publisher for map -> odom (identity for now, SLAM will update)
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Foxglove bridge for visualization
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Elevator controller node
    elevator_controller = Node(
        package='simulation',
        executable='elevator_node',
        name='elevator_controller',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/elevator/call', '/elevator/call'),
            ('/elevator/goto_floor', '/elevator/goto_floor'),
        ]
    )

    # Sim dashboard — live monitor + editable params
    sim_dashboard = Node(
        package='simulation',
        executable='sim_dashboard',
        name='sim_dashboard',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        headless_arg,
        gz_server,
        gz_client,
        spawn_robot,
        gz_bridge,
        static_tf_map_odom,
        foxglove_bridge,
        elevator_controller,
        sim_dashboard,
    ])