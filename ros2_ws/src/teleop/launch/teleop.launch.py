"""Game-pad teleop: joy_node (local device) or a browser pad, then joy_drive.

    ros2 launch teleop teleop.launch.py
    ros2 launch teleop teleop.launch.py joy_source:=browser

joy_source:
  device   a pad on /dev/input; joy_node reads it and publishes /joy.
           Parameters: config/joystick.yaml.
  browser  the pad is plugged into the machine running Foxglove Studio and its
           Joystick panel publishes /joy over the bridge, so there is no local
           device and joy_node is not started. Parameters:
           config/joystick_browser.yaml (a browser reports a different mapping).
           This is the only gamepad path on a Mac - Docker Desktop passes no USB
           through. tooling/ground-up selects it automatically there.

The control scheme (arcade / tank) is the `scheme` parameter inside whichever
config file is selected, not a launch argument.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(ctx, *args, **kwargs):
    share = get_package_share_directory('teleop')
    # .perform(ctx): a bare LaunchConfiguration object is always truthy, so it has
    # to be resolved to its string before it can be compared.
    from_browser = LaunchConfiguration('joy_source').perform(ctx).lower() == 'browser'

    joy_params = os.path.join(
        share, 'config', 'joystick_browser.yaml' if from_browser else 'joystick.yaml')

    nodes = []
    # A browser-sourced pad already arrives on /joy over the Foxglove bridge, so
    # there is no local /dev/input device for joy_node to open.
    if not from_browser:
        nodes.append(Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
        ))

    nodes.append(Node(
        package='teleop',
        executable='joy_drive',
        name='joy_drive',
        parameters=[joy_params],
        remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped')],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_source',
            default_value='device',
            choices=['device', 'browser'],
            description="Where /joy comes from: 'device' (local pad via joy_node) or "
                        "'browser' (Foxglove Joystick panel over the bridge).",
        ),
        OpaqueFunction(function=launch_setup),
    ])
