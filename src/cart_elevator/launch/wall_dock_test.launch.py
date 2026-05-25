"""Closed-loop sim test for the lidar wall-dock controller.

Brings up:
  - fake_cab_sim           (integrates /dock/cmd_vel, publishes /scan of a cab)
  - wall_dock_controller   (fits the cab walls, drives /dock/cmd_vel)

Override the cab size or start pose from the CLI, e.g.:
    ros2 launch cart_elevator wall_dock_test.launch.py \\
        robot_x0:=-0.30 robot_y0:=0.20 robot_yaw0:=0.18 \\
        x_front:=0.65 y_left:=0.65 standoff_x:=0.30

Watch progress with:
    ros2 topic echo /dock/status      # e_x/e_y/e_yaw -> 0, state -> ALIGNED (3)
    ros2 topic echo /dock/cmd_vel
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _arg(name, default):
    return DeclareLaunchArgument(name, default_value=str(default))


def generate_launch_description():
    # Cab geometry + start pose feed the sim.
    sim_keys = ['x_back', 'x_front', 'y_right', 'y_left',
                'robot_x0', 'robot_y0', 'robot_yaw0', 'noise_m']
    sim_defaults = ['-0.7', '0.7', '-0.7', '0.7',
                    '-0.25', '0.18', '0.15', '0.01']

    # Target geometry feeds the controller. side_target 0.70 with a left
    # wall at +0.70 centers the cart; standoff_x is distance to the panel.
    ctrl_keys = ['standoff_x', 'side', 'side_target']
    ctrl_defaults = ['0.30', 'left', '0.70']

    args = [_arg(k, v) for k, v in zip(sim_keys, sim_defaults)]
    args += [_arg(k, v) for k, v in zip(ctrl_keys, ctrl_defaults)]

    sim_params = {k: LaunchConfiguration(k) for k in sim_keys}
    ctrl_params = {k: LaunchConfiguration(k) for k in ctrl_keys}
    # No supervisor in this sim, so boot the controller enabled.
    ctrl_params['autostart'] = True

    return LaunchDescription([
        *args,
        Node(package='cart_elevator', executable='fake_cab_sim',
             name='fake_cab_sim', output='screen',
             parameters=[sim_params]),
        Node(package='cart_elevator', executable='wall_dock_controller',
             name='wall_dock_controller', output='screen',
             parameters=[ctrl_params]),
    ])
