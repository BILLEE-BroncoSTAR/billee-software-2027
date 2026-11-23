"""Launch the Foxglove-ready ZED SDK body-skeleton image overlay."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Return the body-skeleton overlay node launch description."""
    return LaunchDescription([
        Node(
            package="body_tracking",
            executable="zed_body_skeleton_overlay",
            name="zed_body_skeleton_overlay",
            output="screen",
        ),
    ])
