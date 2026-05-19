"""Software-only end-to-end mission demo.

Brings up:
  - cart_supervisor      (the brain; reads mission.yaml)
  - mission_sim_driver   (fakes Nav2, dock, door, floor, press, load
                          buttons so the supervisor sees the whole
                          mission unfold without any hardware)

Use this to verify the BT structure end-to-end before plugging in
real Nav2 + real Limelight + real RoboRIO. Watch progress with:
    ros2 topic echo /mission/state

Mission should complete (IDLE -> ... -> DONE) in roughly 30 s,
gated mostly by the fake delays in mission_sim_driver.

Trigger the start by publishing one Bool:
    ros2 topic pub --once /mission/start std_msgs/Bool "{data: true}"
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mission_yaml = PathJoinSubstitution([
        FindPackageShare('cart_elevator'), 'config', 'mission.yaml'])
    safe_yaml = PathJoinSubstitution([
        FindPackageShare('cart_elevator'), 'config', 'safe.yaml'])
    return LaunchDescription([
        Node(package='cart_elevator', executable='cart_supervisor',
             name='cart_supervisor', output='screen',
             # require_enable=False so the sim doesn't need to publish
             # /mission/enable — keeps the existing one-shot start
             # workflow (publish /mission/start once) working.
             parameters=[mission_yaml, {'require_enable': False}]),
        # Real safe gate — exercises the full retarget + AND path
        # against sim_driver-published door/direction/floor inputs.
        Node(package='cart_elevator', executable='safe_to_enter_gate',
             name='safe_to_enter_gate', output='screen',
             parameters=[safe_yaml]),
        Node(package='cart_elevator', executable='mission_sim_driver',
             name='mission_sim_driver', output='screen',
             parameters=[{'target_floor': 4}]),
    ])
