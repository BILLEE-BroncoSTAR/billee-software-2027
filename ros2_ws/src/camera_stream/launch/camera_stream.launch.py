"""CSI cameras + a Foxglove bridge on :8765, without the rest of the rover.

    ros2 launch camera_stream camera_stream.launch.py                # all five cameras
    ros2 launch camera_stream camera_stream.launch.py source:=test   # test pattern

rover.launch.py already starts the cameras and a bridge; use this only to look at the cameras
alone. Every argument of csi_cameras.launch.py passes through (see its --show-args).
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("camera_stream"), "launch",
                                  "csi_cameras.launch.py"])))
    # Launched directly, as in chassis_bringup/launch/viz.launch.py: the bridge's own launch
    # file only sets defaults and pulls in the launch_xml frontend.
    bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        parameters=[{"port": 8765, "address": "0.0.0.0", "topic_whitelist": [".*"],
                     "send_buffer_limit": 10000000}],
        output="screen",
    )
    return LaunchDescription([cameras, bridge])
