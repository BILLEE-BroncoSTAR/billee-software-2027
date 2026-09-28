"""Launch the synthetic VIO publisher used with Gazebo ground truth."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("zed2i"), "config", "synthetic_vio.yaml"]
    )
    return LaunchDescription([
        DeclareLaunchArgument("profile", default_value="validation"),
        DeclareLaunchArgument("seed", default_value="42"),
        DeclareLaunchArgument("truth_topic", default_value="/gz/odom"),
        Node(
            package="zed2i",
            executable="synthetic_vio_node",
            name="synthetic_vio",
            output="screen",
            parameters=[
                params_file,
                {
                    "use_sim_time": True,
                    "profile": LaunchConfiguration("profile"),
                    "seed": LaunchConfiguration("seed"),
                    "truth_topic": LaunchConfiguration("truth_topic"),
                },
            ],
        ),
    ])
