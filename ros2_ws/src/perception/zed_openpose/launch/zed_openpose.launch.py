"""zed_openpose_node alone: ZED 2 + CUDA OpenPose -> /people/skeletons, markers, debug image.

    ros2 launch zed_openpose zed_openpose.launch.py [svo_path:=/path/rec.svo2] [model_pose:=COCO]

Usually started through `ros2 launch people people_rover.launch.py`, which adds the gesture node,
robot_state_publisher and foxglove_bridge.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_folder",
            # The trailing "" keeps the "/" OpenPose expects at the end of the folder.
            default_value=PathJoinSubstitution(
                [EnvironmentVariable("PIXI_PROJECT_ROOT", default_value="."), "models", "openpose", ""]),
            description="OpenPose models (pose/body_25, pose/coco); see "
            "people/scripts/get_openpose_models.sh.",
        ),
        DeclareLaunchArgument("model_pose", default_value="BODY_25"),
        DeclareLaunchArgument("svo_path", default_value="",
                              description="Replay a ZED recording instead of the live camera."),
        DeclareLaunchArgument("target_frame", default_value="base_link"),
        Node(
            package="zed_openpose",
            executable="zed_openpose_node",
            name="zed_openpose",
            output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare("zed_openpose"), "config", "zed_openpose.yaml"]),
                {
                    "model_folder": LaunchConfiguration("model_folder"),
                    "model_pose": LaunchConfiguration("model_pose"),
                    "svo_path": LaunchConfiguration("svo_path"),
                    "target_frame": LaunchConfiguration("target_frame"),
                },
            ],
        ),
    ])
