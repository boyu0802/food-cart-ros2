"""Closed-loop sim test for the AprilTag docking controller.

Brings up:
  - fake_dock_sim        (integrates cmd_vel, publishes /limelight/tag)
  - dock_controller      (consumes /limelight/tag, drives /dock/cmd_vel)

Override the start pose or tag pose from the CLI, e.g.:
    ros2 launch cart_elevator dock_test.launch.py \\
        robot_x0:=-0.5 robot_yaw0:=0.3 tag_x:=2.0 tag_y:=-0.4

Watch progress with:
    ros2 topic echo /dock/cmd_vel
    ros2 topic echo /limelight/tag
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _arg(name, default):
    return DeclareLaunchArgument(name, default_value=str(default))


def generate_launch_description():
    sim_keys = ['robot_x0', 'robot_y0', 'robot_yaw0',
                'tag_x', 'tag_y', 'tag_yaw',
                'fov_half_deg', 'max_range_m']
    sim_defaults = ['0.0', '0.0', '0.0',
                    '1.5', '0.3', '3.14159265',
                    '30.0', '6.0']

    ctrl_keys = ['target_tag_id', 'standoff_x', 'standoff_y']
    ctrl_defaults = ['101', '0.6', '0.0']

    args = [_arg(k, v) for k, v in zip(sim_keys, sim_defaults)]
    args += [_arg(k, v) for k, v in zip(ctrl_keys, ctrl_defaults)]

    sim_params = {k: LaunchConfiguration(k) for k in sim_keys}
    ctrl_params = {k: LaunchConfiguration(k) for k in ctrl_keys}

    return LaunchDescription([
        *args,
        Node(package='cart_elevator', executable='fake_dock_sim',
             name='fake_dock_sim', output='screen',
             parameters=[sim_params]),
        Node(package='cart_elevator', executable='dock_controller',
             name='dock_controller', output='screen',
             parameters=[ctrl_params]),
    ])
