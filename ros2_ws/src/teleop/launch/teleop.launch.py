"""Gamepad teleop: joy_node + joy_tank_drive -> /diff_drive_controller/cmd_vel_unstamped."""

# ros2 launch teleop teleop.launch.py                      # scheme from config/joystick.yaml
# ros2 launch teleop teleop.launch.py drive_scheme:=tank   # or arcade

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    joy_params = os.path.join(get_package_share_directory('teleop'), 'config', 'joystick.yaml')
    scheme = LaunchConfiguration('drive_scheme').perform(context).strip()
    if scheme not in ('', 'arcade', 'tank'):
        raise RuntimeError(
            f'teleop.launch.py: drive_scheme must be arcade or tank, got {scheme!r}')
    teleop_params = [joy_params] + ([{'scheme': scheme}] if scheme else [])

    return [
        Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
        ),
        Node(
            package='teleop',
            executable='joy_tank_drive',
            name='joy_tank_drive',
            parameters=teleop_params,
            remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drive_scheme',
            default_value='',
            description="arcade (left stick steers, RT forward, LT reverse) or tank (left/right "
            "stick Y = left/right wheels). Empty = 'scheme' in config/joystick.yaml.",
        ),
        OpaqueFunction(function=_nodes),
    ])
