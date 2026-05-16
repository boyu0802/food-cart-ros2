"""Blind floor-detection comparison run.

Launches:
  - fake_elevator_imu  (synthetic /imu, scripted elevator trips)
  - fake_tag_publisher (synthetic /limelight/tag, scripted tag visibility)
  - floor_v1_apriltag, floor_v2_time, floor_v3_accel, floor_v4_fusion

Look at the four /elevator/floor_v* topics + /elevator/floor in Foxglove
(or `ros2 topic echo`) to compare the detectors on identical synthetic
data. Swap the scenario param to test different trip shapes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('cart_elevator'), 'config', 'floor.yaml'])

    imu_scenario = LaunchConfiguration('imu_scenario')
    tag_scenario = LaunchConfiguration('tag_scenario')

    return LaunchDescription([
        DeclareLaunchArgument('imu_scenario', default_value='+3'),
        DeclareLaunchArgument(
            'tag_scenario',
            default_value='2:101:0.04; 8:103:0.05; 14:104:0.06'),

        Node(package='cart_elevator', executable='fake_elevator_imu',
             name='fake_elevator_imu', output='screen',
             parameters=[params_file, {'scenario': imu_scenario}]),

        Node(package='cart_elevator', executable='fake_tag_publisher',
             name='fake_tag_publisher', output='screen',
             parameters=[params_file, {'scenario': tag_scenario}]),

        Node(package='cart_elevator', executable='floor_v1_apriltag',
             name='floor_v1_apriltag', output='screen',
             parameters=[params_file]),
        Node(package='cart_elevator', executable='floor_v2_time',
             name='floor_v2_time', output='screen',
             parameters=[params_file]),
        Node(package='cart_elevator', executable='floor_v3_accel',
             name='floor_v3_accel', output='screen',
             parameters=[params_file]),
        Node(package='cart_elevator', executable='floor_v4_fusion',
             name='floor_v4_fusion', output='screen',
             parameters=[params_file]),
    ])
