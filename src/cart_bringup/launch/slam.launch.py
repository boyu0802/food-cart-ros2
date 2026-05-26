"""Run slam_toolbox in MAPPING (online async) or LOCALIZATION mode.

mapping (default):  async_slam_toolbox_node builds a fresh map and publishes
                    /map + the map->odom TF.
localization:       localization_slam_toolbox_node deserializes a saved
                    posegraph (map:=/path/to/map, no extension), publishes the
                    loaded /map + map->odom, and registers the live /scan
                    against it. No new map is created.

Requires (both modes):
  - /scan from the lidar driver (via scan_sector_filter)
  - odom -> base_footprint TF (published by nt_bridge once RoboRIO is up)
  - base_footprint -> base_link -> laser TF from cart_description / RSP

On Jazzy these are LIFECYCLE nodes, so a nav2_lifecycle_manager (autostart)
configures+activates whichever one we launch. The node is named 'slam_toolbox'
in both modes so the manager config is identical. Without the manager it sits
unconfigured and never creates /scan or /map.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_mode = LaunchConfiguration('slam_mode')
    slam_params = LaunchConfiguration('slam_params_file')
    map_file = LaunchConfiguration('map')

    mapping_params = PathJoinSubstitution([
        FindPackageShare('cart_bringup'), 'config', 'slam_toolbox.yaml'])
    loc_params = PathJoinSubstitution([
        FindPackageShare('cart_bringup'), 'config',
        'slam_toolbox_localization.yaml'])
    default_map = PathJoinSubstitution([
        FindPackageShare('cart_bringup'), 'maps', 'floor_4', 'map'])

    is_mapping = PythonExpression(["'", slam_mode, "' == 'mapping'"])
    is_localization = PythonExpression(["'", slam_mode, "' == 'localization'"])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'slam_mode', default_value='mapping',
            description="'mapping' (build a new map) or 'localization' "
                        "(load map:= and localize against it)."),
        DeclareLaunchArgument(
            'slam_params_file', default_value=mapping_params,
            description='slam_toolbox MAPPING-mode YAML.'),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Localization only: serialized map path WITHOUT '
                        'extension (slam_toolbox appends .posegraph/.data). '
                        'Defaults to the floor_4 map.'),

        # ---- MAPPING ----
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params, {'use_sim_time': use_sim_time}],
            condition=IfCondition(is_mapping),
        ),

        # ---- LOCALIZATION ----
        # map_file_name is appended after the YAML so it overrides the file's
        # default — the same config localizes against any floor via map:=.
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[loc_params, {
                'use_sim_time': use_sim_time,
                'map_file_name': map_file,
            }],
            condition=IfCondition(is_localization),
        ),

        # Lifecycle manager — same node name ('slam_toolbox') in both modes.
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
