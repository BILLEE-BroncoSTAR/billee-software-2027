from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # --- Launch arguments ---
    bitrate = LaunchConfiguration("bitrate")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    fps = LaunchConfiguration("fps")

    declare_args = [
        DeclareLaunchArgument(
            "bitrate",
            default_value="3000000",
            description="Per-camera bitrate in bits/sec.",
        ),
        DeclareLaunchArgument(
            "width",
            default_value="1280",
            description="Video width in pixels.",
        ),
        DeclareLaunchArgument(
            "height",
            default_value="720",
            description="Video height in pixels.",
        ),
        DeclareLaunchArgument(
            "fps",
            default_value="30",
            description="Frames per second.",
        ),
    ]

    # --- Foxglove Bridge ---
    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            ])
        )
    )

    # --- Six CSI Compressed Videos ---
    six_csi = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("camera_stream"),
                "launch",
                "six_csi_compressed_videos.launch.py",
            ])
        ),
        launch_arguments={
            "bitrate": bitrate,
            "width": width,
            "height": height,
            "fps": fps,
        }.items(),
    )

    return LaunchDescription(
        declare_args
        + [
            foxglove_bridge,
            six_csi,
        ]
    )
