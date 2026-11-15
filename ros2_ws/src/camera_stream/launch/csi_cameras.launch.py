"""The rover's CSI cameras: one csi_compressed_video_pub per camera.

    ros2 launch camera_stream csi_cameras.launch.py                 # all five
    ros2 launch camera_stream csi_cameras.launch.py cameras:=3      # cam0..cam2
    ros2 launch camera_stream csi_cameras.launch.py source:=test    # test pattern, no cameras

rover.launch.py includes this (csi_cameras:=5 by default). Camera N is Argus sensor-id N and
publishes
    /camN/video                     foxglove_msgs/CompressedVideo, H.264 (Foxglove Image panel)
    /camN/image_raw/compressed      sensor_msgs/CompressedImage, JPEG (RViz Image display)

Argus numbers the cameras it found at boot 0, 1, 2... in device-tree port order, so a camera
missing from a lower port shifts the ones after it down. A camera with nothing behind its
sensor-id publishes a "NO CAMERA" placeholder instead and checks again every 10 s; it never
stops the launch.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Index = Argus sensor-id = /camN. Names match the Foxglove dashboard panels
# (chassis_bringup/foxglove/rover_dashboard.json) and the RViz displays.
# flip: how the camera is mounted, fixed in the Jetson's video hardware before encoding, so
# Foxglove and RViz both get the corrected image (nvvidconv flip-method):
#   0 as is   2 rotate 180 (mounted upside down)   4 mirror left-right   6 mirror top-bottom
#   1 / 3 rotate 90 counter-/clockwise (the stream becomes portrait, 720x1280)
# Change it here and relaunch; nothing to rebuild with a --symlink-install build.
CAMERAS = (
    # name, flip
    ("front", 2),       # mounted upside down
    ("rear_left", 2),   # mounted upside down
    ("rear_right", 0),
    ("top_front", 0),
    ("reverse", 0),
)

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
    DeclareLaunchArgument("fps", default_value="30", description="Camera and H.264 frame rate."),
    DeclareLaunchArgument(
        "bitrate", default_value="3000000",
        description="H.264 bits/s per camera (Foxglove). 5 x 3 Mbit/s fits on Wi-Fi."),
    DeclareLaunchArgument(
        "jpeg_fps", default_value="10",
        description="JPEG frame rate per camera (RViz); 0 = no JPEG topics."),
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
        "source", "width", "height", "fps", "bitrate", "jpeg_fps", "jpeg_width", "jpeg_height")}
    return [
        Node(
            package="camera_stream",
            executable="csi_compressed_video_pub",
            name=f"cam{i}_{name}",
            output="screen",
            # A crash (a bug, not a missing camera: that's handled inside) restarts the node.
            respawn=True,
            respawn_delay=5.0,
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
