"""Launch Nav2 (planner, MPPI controller, behaviors, BT navigator, costmaps).

Map source is slam_toolbox (live), so we do NOT start map_server or amcl.
If you later want pure-localization mode against a saved map, pass
use_slam:=false and provide map:=/path/to/map.yaml — handled in bringup.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    default_params = PathJoinSubstitution([
        FindPackageShare('cart_bringup'), 'config', 'nav2.yaml'])

    # Lifecycle-managed Nav2 nodes (no map_server / amcl — slam_toolbox
    # publishes the map and the map->odom TF itself).
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    common_params = [params_file, {'use_sim_time': use_sim_time}]

    nav2_nodes = GroupAction([
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen', parameters=common_params,
             remappings=[('cmd_vel', 'cmd_vel_nav')]),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', output='screen', parameters=common_params),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen', parameters=common_params),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen', parameters=common_params),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen', parameters=common_params),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', output='screen', parameters=common_params),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', output='screen', parameters=common_params,
             remappings=[('cmd_vel', 'cmd_vel_nav'),
                         ('cmd_vel_smoothed', 'cmd_vel')]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }],
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        nav2_nodes,
    ])
