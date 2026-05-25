"""Publish the food cart URDF via robot_state_publisher.

Use this on its own when you just want TFs (e.g. to look at the robot in
Foxglove without running the full nav stack), or include it from
cart_bringup's bringup.launch.py.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_joint_states = LaunchConfiguration('publish_joint_states')

    urdf_path = PathJoinSubstitution([
        FindPackageShare('cart_description'),
        'urdf',
        'cart.urdf.xacro',
    ])
    robot_description = {
        # Wrap in ParameterValue(str) so launch treats the xacro output as a
        # string, not YAML (URDF XML isn't valid YAML -> "Unable to parse").
        'robot_description': ParameterValue(
            Command(['xacro ', urdf_path]), value_type=str),
        'use_sim_time': use_sim_time,
    }

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'publish_joint_states', default_value='true',
            description='Run joint_state_publisher (harmless even with no joints)'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=None,  # always start for now; cheap
        ),
    ])
