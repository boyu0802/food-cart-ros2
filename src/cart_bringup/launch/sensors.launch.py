"""Bring up the Slamtec C1 lidar and the Intel RealSense D455.

Notes:
- The D455 IMU is NOT enabled (kernel doesn't ship hid-sensor-hub on the
  rockchip 6.1 kernel — driver spams errors if asked).
- The realsense pointcloud has a quirky runtime param name on this build;
  we set it via a node param override below. See docs/SETUP.md.
- The C1 is mounted centered among the cart's 4 corner posts, so its raw
  scan is published on /scan_raw and passed through scan_sector_filter,
  which masks the post bearings and republishes the clean scan on /scan.
  Everything downstream (slam_toolbox, both Nav2 costmaps) keeps consuming
  /scan unchanged. Sectors live in config/scan_filter.yaml.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_camera = LaunchConfiguration('enable_camera')
    enable_lidar = LaunchConfiguration('enable_lidar')

    rplidar_launch = PathJoinSubstitution([
        FindPackageShare('rplidar_ros'), 'launch', 'rplidar_c1_launch.py'])
    rs_launch = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
    scan_filter_config = PathJoinSubstitution([
        FindPackageShare('cart_bringup'), 'config', 'scan_filter.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),

        # ---- lidar: driver -> /scan_raw, then sector filter -> /scan ----
        # SetRemap inside the group reroutes the rplidar_node's 'scan'
        # output to /scan_raw; scan_sector_filter masks the corner-post
        # sectors and republishes the clean /scan that everyone consumes.
        GroupAction(
            condition=IfCondition(enable_lidar),
            actions=[
                SetRemap(src='scan', dst='scan_raw'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rplidar_launch)),
            ],
        ),
        Node(
            package='cart_bringup',
            executable='scan_sector_filter',
            name='scan_sector_filter',
            output='screen',
            parameters=[scan_filter_config],
            condition=IfCondition(enable_lidar),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            condition=IfCondition(enable_camera),
            launch_arguments={
                # Explicitly DO NOT enable gyro/accel — see docs/SETUP.md.
                'enable_gyro': 'false',
                'enable_accel': 'false',
                'enable_color': 'true',
                'enable_depth': 'true',
                # NOTE: 'pointcloud.enable:=true' does NOT propagate on this
                # arm64 build. After launch, run:
                #   ros2 param set /camera/camera pointcloud__neon_.enable true
                'pointcloud.enable': 'true',
                'align_depth.enable': 'true',
            }.items(),
        ),
    ])
