from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="true for Gazebo or a bag played with --clock; false on the real rover.",
        ),
        DeclareLaunchArgument("net_height", default_value="368",
                              description="OpenPose input height (px); lower = faster, less accurate."),
        Node(
            package="people",
            executable="openpose_node",
            output="screen",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "net_height": LaunchConfiguration("net_height"),
            }],
        ),
    ])
