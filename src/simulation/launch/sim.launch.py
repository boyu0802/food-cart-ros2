"""
Launch file for Gazebo simulation with school world and food cart robot.
Starts Gazebo, spawns the robot, ros_gz_bridge, and foxglove_bridge.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value='school.sdf',
        description='Gazebo world file name'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless (no GUI)'
    )

    sim_share = FindPackageShare('simulation')
    world_path = PathJoinSubstitution([sim_share, 'worlds', LaunchConfiguration('world')])
    robot_model_path = PathJoinSubstitution([sim_share, 'models', 'food_cart', 'model.sdf'])
    bridge_config_path = PathJoinSubstitution([sim_share, 'config', 'gz_bridge.yaml'])
    urdf_path = PathJoinSubstitution([FindPackageShare('cart_description'), 'urdf', 'cart.urdf.xacro'])

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world_path],
        output='screen',
        name='gz_sim_server'
    )

    # GUI only when NOT headless
    gz_client = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        name='gz_sim_client',
        condition=UnlessCondition(LaunchConfiguration('headless'))
    )

    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-file', robot_model_path,
            '-name', 'food_cart',
            '-x', '-8.0', '-y', '3.0', '-z', '11.4',
            '-Y', '1.57'
        ],
        output='screen',
        name='spawn_robot'
    )

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

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

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

    elevator_controller = Node(
        package='simulation',
        executable='elevator_node',
        name='elevator_controller',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

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
        robot_state_publisher,
        static_tf_map_odom,
        foxglove_bridge,
        elevator_controller,
        sim_dashboard,
    ])
