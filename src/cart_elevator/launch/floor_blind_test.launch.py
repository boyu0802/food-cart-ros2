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
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('cart_elevator'), 'config', 'floor.yaml'])

    imu_scenario = LaunchConfiguration('imu_scenario')
    tag_scenario = LaunchConfiguration('tag_scenario')
    imu_bias_mss = LaunchConfiguration('imu_bias_mss')
    imu_noise_std_mss = LaunchConfiguration('imu_noise_std_mss')
    starting_floor = LaunchConfiguration('starting_floor')

    return LaunchDescription([
        DeclareLaunchArgument('imu_scenario', default_value='+3'),
        DeclareLaunchArgument(
            'tag_scenario',
            default_value='2:101:0.04; 8:103:0.05; 14:104:0.06'),
        DeclareLaunchArgument('imu_bias_mss', default_value='0.0',
                              description='Constant accel-z bias for fake_imu (m/s^2).'),
        DeclareLaunchArgument('imu_noise_std_mss', default_value='0.02',
                              description='Gaussian noise stddev for fake_imu accel (m/s^2).'),
        DeclareLaunchArgument('starting_floor', default_value='1',
                              description='Starting floor for both fake_imu ground truth and v2/v3 estimators.'),

        Node(package='cart_elevator', executable='fake_elevator_imu',
             name='fake_elevator_imu', output='screen',
             parameters=[params_file, {
                 'scenario': ParameterValue(imu_scenario, value_type=str),
                 'bias_mss': ParameterValue(imu_bias_mss, value_type=float),
                 'noise_std_mss': ParameterValue(imu_noise_std_mss, value_type=float),
                 'starting_floor': ParameterValue(starting_floor, value_type=int),
             }]),

        Node(package='cart_elevator', executable='fake_tag_publisher',
             name='fake_tag_publisher', output='screen',
             parameters=[params_file, {
                 'scenario': ParameterValue(tag_scenario, value_type=str),
             }]),

        Node(package='cart_elevator', executable='floor_v1_apriltag',
             name='floor_v1_apriltag', output='screen',
             parameters=[params_file]),
        Node(package='cart_elevator', executable='floor_v2_time',
             name='floor_v2_time', output='screen',
             parameters=[params_file, {
                 'starting_floor': ParameterValue(starting_floor, value_type=int),
             }]),
        Node(package='cart_elevator', executable='floor_v3_accel',
             name='floor_v3_accel', output='screen',
             parameters=[params_file, {
                 'starting_floor': ParameterValue(starting_floor, value_type=int),
             }]),
        Node(package='cart_elevator', executable='floor_v4_fusion',
             name='floor_v4_fusion', output='screen',
             parameters=[params_file]),
    ])
