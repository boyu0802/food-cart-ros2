"""End-to-end test for direction_v1_digit against a recorded elevator clip.

Launches:
  - video_replay              (publishes the recorded mp4 on /image)
  - direction_v1_digit        (digit-tracking detector under test)
  - direction_v3_fusion       (fuses v1_digit + v2_flow; v2 will be UNKNOWN)
  - direction_debug_view      (cv2 window overlay; disable with show:=false)

By default replays test/data/elevator_1_to_4_up.mp4 (floors 1->4 ascending).
Override the clip with:
    ros2 launch cart_elevator direction_video_test.launch.py \\
        video_path:=/abs/path/to/other.mp4

Run headless (no display) with:
    ros2 launch cart_elevator direction_video_test.launch.py show:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('cart_elevator'), 'config', 'floor.yaml'])

    default_video = PathJoinSubstitution([
        FindPackageShare('cart_elevator'), 'test', 'data',
        'elevator_1_to_4_up.mp4'])

    video_path = LaunchConfiguration('video_path')
    show = LaunchConfiguration('show')

    return LaunchDescription([
        DeclareLaunchArgument('video_path', default_value=default_video),
        DeclareLaunchArgument('show', default_value='true',
                              description='Pop the cv2 debug window'),

        Node(package='cart_elevator', executable='video_replay',
             name='video_replay', output='screen',
             parameters=[params_file, {'video_path': video_path}]),

        Node(package='cart_elevator', executable='direction_v1_digit',
             name='direction_v1_digit', output='screen',
             parameters=[params_file]),
        Node(package='cart_elevator', executable='direction_v3_fusion',
             name='direction_v3_fusion', output='screen',
             parameters=[params_file]),
        Node(package='cart_elevator', executable='direction_debug_view',
             name='direction_debug_view', output='screen',
             parameters=[params_file],
             condition=IfCondition(show)),
    ])
