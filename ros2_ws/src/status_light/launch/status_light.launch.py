"""The mode light on its own (rover.launch.py starts it with status_light:=<pattern>).

    ros2 launch status_light status_light.launch.py pattern:=idle
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pattern', default_value='idle',
                              description='idle | teleop | autonomous | arrived | off'),
        DeclareLaunchArgument('serial', default_value='',
                              description='blink(1) serial number (default: the first one)'),
        Node(package='status_light', executable='status_light', name='status_light',
             output='screen',
             parameters=[{'pattern': LaunchConfiguration('pattern'),
                          'serial': LaunchConfiguration('serial')}]),
    ])
