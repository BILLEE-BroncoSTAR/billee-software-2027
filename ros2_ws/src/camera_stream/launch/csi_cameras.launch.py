"""The rover's CSI cameras: one csi_compressed_video_pub per camera.

    ros2 launch camera_stream csi_cameras.launch.py                 # all cameras in the config
    ros2 launch camera_stream csi_cameras.launch.py cameras:=3      # cam0..cam2
    ros2 launch camera_stream csi_cameras.launch.py source:=test    # test pattern, no cameras

rover.launch.py always includes this, with every camera in the config. Camera N is Argus
sensor-id N and publishes
    /camN/video                     foxglove_msgs/CompressedVideo, H.264 (Foxglove Image panel)
    /camN/image_raw/compressed      sensor_msgs/CompressedImage, JPEG (RViz Image display)

The camera list (name and flip for each camera) lives in config/cameras.yaml in this package.
Index = Argus sensor-id = /camN: the first entry is cam0, the second cam1, and so on. Names
match the Foxglove dashboard panels (chassis_bringup/foxglove/rover_dashboard.json) and the
RViz displays. Change the file and relaunch; nothing to rebuild with a --symlink-install build.

Argus numbers the cameras it found at boot 0, 1, 2... in device-tree port order, so a camera
missing from a lower port shifts the ones after it down. A camera with nothing behind its
sensor-id publishes a "NO CAMERA" placeholder instead and checks again every 10 s; it never
stops the launch.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAMERAS_CONFIG = os.path.join(
    get_package_share_directory("camera_stream"), "config", "cameras.yaml")


def _load_cameras():
    """Return [(name, flip), ...] in file order. Index = Argus sensor-id = /camN."""
    with open(CAMERAS_CONFIG) as f:
        data = yaml.safe_load(f) or {}
    entries = data.get("cameras")
    if not entries:
        raise RuntimeError(f"csi_cameras.launch.py: no 'cameras' section in {CAMERAS_CONFIG}")
    return [
        (str(cfg.get("name", key)), int(cfg.get("flip", 0)))
        for key, cfg in entries.items()
    ]


# Index = Argus sensor-id = /camN, exactly as the old in-file CAMERAS tuple.
CAMERAS = _load_cameras()

declare_args = [
    DeclareLaunchArgument(
        "cameras", default_value=str(len(CAMERAS)),
        description=f"How many CSI cameras to start, cam0 upwards (0-{len(CAMERAS)}): "
        + ", ".join(f"cam{i} {name}" for i, (name, _) in enumerate(CAMERAS)) + "."),
    DeclareLaunchArgument(
        "source", default_value="argus", choices=["argus", "test"],
        description="argus = the cameras; test = a moving test pattern through the same "
        "encoders (no cameras needed)."),
    DeclareLaunchArgument("width", default_value="1280", description="H.264 width."),
    DeclareLaunchArgument("height", default_value="720", description="H.264 height."),
    # Defaults are for the long-range radio link: as few bits as a usable teleop view needs.
    # On a LAN, e.g. fps:=30 bitrate:=3000000 min_qp:=22 for the old near-lossless streams.
    DeclareLaunchArgument("fps", default_value="15", description="Camera and H.264 frame rate."),
    DeclareLaunchArgument(
        "bitrate", default_value="1000000",
        description="H.264 cap per camera, bits/s (Foxglove). A calm scene uses far less "
        "(min_qp); a busy one never more. 5 x 1 Mbit/s for the radio link."),
    DeclareLaunchArgument(
        "min_qp", default_value="28",
        description="H.264 quality floor (0-51, lower = finer): the encoder spends no bits "
        "beyond this quality. 28 is clean for teleop at about half the bits of 22 (close to "
        "visually lossless); 0 = no floor (always ~bitrate)."),
    DeclareLaunchArgument(
        "jpeg_fps", default_value="5",
        description="JPEG frame rate per camera (RViz; JPEG costs ~5x the bits of H.264, so use "
        "Foxglove over the radio); 0 = no JPEG topics."),
    DeclareLaunchArgument("jpeg_width", default_value="640", description="JPEG width."),
    DeclareLaunchArgument("jpeg_height", default_value="360", description="JPEG height."),
    DeclareLaunchArgument(
        "use_sim_time", default_value="false",
        description="Stamp frames with /clock (rover.launch.py sets it in mode:=sim)."),
]


def _cameras(context):
    count = int(LaunchConfiguration("cameras").perform(context))
    if not 0 <= count <= len(CAMERAS):
        raise RuntimeError(f"csi_cameras.launch.py: cameras:={count}, expected 0-{len(CAMERAS)}")
    shared = {key: LaunchConfiguration(key) for key in (
        "source", "width", "height", "fps", "bitrate", "min_qp", "jpeg_fps", "jpeg_width",
        "jpeg_height")}
    return [
        Node(
            package="camera_stream",
            executable="csi_compressed_video_pub",
            name=f"cam{i}_{name}",
            output="screen",
            # A crash (a bug, not a missing camera: that's handled inside) restarts the node.
            respawn=True,
            respawn_delay=5.0,
            # Ctrl-C: give the node time to close its camera (STOP_TIMEOUT_S = 10 s in the
            # node) before launch escalates; the default 5 s cuts Argus off mid-close.
            sigterm_timeout="15",
            sigkill_timeout="5",
            parameters=[{
                **shared,
                "name": name,
                "sensor_id": i,
                "flip_method": flip,
                "video_topic": f"/cam{i}/video",
                "jpeg_topic": f"/cam{i}/image_raw/compressed",
                "frame_id": f"cam{i}_optical",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        )
        for i, (name, flip) in enumerate(CAMERAS[:count])
    ]


def generate_launch_description():
    return LaunchDescription(declare_args + [OpaqueFunction(function=_cameras)])
