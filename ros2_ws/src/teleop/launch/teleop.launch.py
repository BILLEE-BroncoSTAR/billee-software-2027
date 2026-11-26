from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    joy_params = os.path.join(get_package_share_directory('teleop'), 'config', 'joystick.yaml')

    cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/diff_drive_controller/cmd_vel_unstamped',
        description='Where joy_tank_drive publishes. The default drives diff_drive_controller '
        'directly; /cmd_vel_joy feeds the command mux (chassis_bringup cmd_mux:=true).',
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params]
    )

    teleop_node = Node(
        package='teleop',
        executable='joy_tank_drive',
        name='joy_tank_drive',
        parameters=[joy_params],
        remappings=[('/cmd_vel', LaunchConfiguration('cmd_vel_topic'))]
    )

    return LaunchDescription([
        cmd_vel_topic,
        joy_node,
        teleop_node
    ])
