"""Bring up the Slamtec C1 lidar and the Intel RealSense D455.

Notes:
- The D455 IMU is NOT enabled (kernel doesn't ship hid-sensor-hub on the
  rockchip 6.1 kernel — driver spams errors if asked).
- The realsense pointcloud has a quirky runtime param name on this build;
  we set it via a node param override below. See docs/SETUP.md.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_camera = LaunchConfiguration('enable_camera')
    enable_lidar = LaunchConfiguration('enable_lidar')

    rplidar_launch = PathJoinSubstitution([
        FindPackageShare('rplidar_ros'), 'launch', 'rplidar_c1_launch.py'])
    rs_launch = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
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
