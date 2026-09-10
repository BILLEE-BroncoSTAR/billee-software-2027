"""Viewers for the BILLEE drivetrain: RViz2 and/or the Foxglove WebSocket bridge.

Standalone (both viewers on by default):

    ros2 launch chassis_bringup viz.launch.py

Alongside a running sim or real bring-up you usually just pass the flags to that
launch instead:

    ros2 launch chassis_bringup sim_gz.launch.py rviz:=true foxglove:=true
    ros2 launch chassis_bringup real.launch.py  can_interface:=mock rviz:=true

Both readers show the same thing: the robot model + TF tree + wheel odometry,
fixed frame `odom`. RViz needs a display (use `xvfb-run -a` for headless checks);
Foxglove Studio connects from a laptop to `ws://<jetson-ip>:8765` and needs none.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")
    foxglove = LaunchConfiguration("foxglove")
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    rviz_config = LaunchConfiguration("rviz_config")

    declare_args = [
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Start RViz2."
        ),
        DeclareLaunchArgument(
            "foxglove",
            default_value="true",
            description="Start the foxglove_bridge WebSocket server on :8765.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Set true when viewing the Gazebo sim so TF timestamps line up.",
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("chassis_bringup"), "rviz", "drivetrain.rviz"]
            ),
            description="RViz2 .rviz config file.",
        ),
    ]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(rviz),
    )

    # foxglove_bridge node launched directly (its shipped launch file only sets
    # defaults, and going through it would pull in the launch_xml frontend).
    foxglove_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[
            {
                "port": 8765,
                "address": "0.0.0.0",
                "use_sim_time": use_sim_time,
                "topic_whitelist": [".*"],
                "send_buffer_limit": 10000000,
            }
        ],
        output="screen",
        condition=IfCondition(foxglove),
    )

    return LaunchDescription(declare_args + [rviz_node, foxglove_node])
