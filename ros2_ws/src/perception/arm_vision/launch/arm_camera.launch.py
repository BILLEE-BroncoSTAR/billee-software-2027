"""The arm's RealSense D435i: realsense2_camera, ArUco, keyboard (+ arm TF, Foxglove bridge).

    # on its own (Jetson, l4t container): camera + ArUco + foxglove_bridge on :8765
    ros2 launch arm_vision arm_camera.launch.py

    # inside rover.launch.py (arm_camera:=auto|true): the drivetrain already runs the bridge
    ros2 launch arm_vision arm_camera.launch.py standalone:=false

    # markers: dictionary and printed size (m) of the black square; per-id sizes (the keyboard's
    # markers: aruco_node and keyboard_node both use them)
    ros2 launch arm_vision arm_camera.launch.py dictionary:=DICT_4X4_50 marker_size:=0.1
    ros2 launch arm_vision arm_camera.launch.py marker_sizes:=10-13:0.04

    # camera only / ArUco only (e.g. against a rosbag of /arm_cam/*)
    ros2 launch arm_vision arm_camera.launch.py aruco:=false
    ros2 launch arm_vision arm_camera.launch.py camera:=false

    # keyboard: relative to the rover (default, needs the arm's TF) or just to the camera
    ros2 launch arm_vision arm_camera.launch.py                          # arm_tf:=auto
    ros2 launch arm_vision arm_camera.launch.py arm_tf:=zero             # bench: arm not running
    ros2 launch arm_vision arm_camera.launch.py target_frame:=           # camera frame only

    # no arm URDF at all: a fixed camera mount instead (m, rad); turns arm_tf off
    ros2 launch arm_vision arm_camera.launch.py parent_frame:=base_link mount_z:=0.5

Topics (/arm_cam = the realsense2_camera node; config/realsense_d435i.yaml)
  /arm_cam/color/image_raw (+ /compressed)               colour, 640x480 @ 30, rgb8
  /arm_cam/color/camera_info
  /arm_cam/aligned_depth_to_color/image_raw (+ /compressedDepth)   depth on the colour pixels, mm
  /arm_cam/depth/image_rect_raw                          raw depth, 848x480
  /arm_cam/imu                                           gyro + accel (D435i)
  /arm_cam/aruco/markers                                 arm_vision_msgs/ArucoMarkers
  /arm_cam/aruco/image/compressed                        annotated JPEG: the ground station's feed
  /arm_cam/aruco/visualization                           MarkerArray for 3D panels
  /arm_cam/keyboard/keys                                 arm_vision_msgs/Keyboard: every key
  /arm_cam/keyboard/pose                                 PoseStamped in target_frame
  /arm_cam/keyboard/visualization                        MarkerArray (keyboard frame)
TF: arm_cam_link -> arm_cam_{color,depth,...}_optical_frame (driver), optical frame -> aruco_<id>,
    target_frame -> keyboard (keyboard_node), base_link -> ... -> arm_cam_link (arm_tf: the arm's
    robot_state_publisher, arm_description/launch/arm_description.launch.py).

realsense2_camera waits for the camera and reconnects after a USB drop, so the launch can start
before the camera is plugged in. Foxglove layout: chassis_bringup/foxglove/rover_arm.json.
"""

import os

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from arm_vision.aruco import format_marker_sizes, parse_marker_sizes
from arm_vision.usb import camera_warnings, realsense_cameras
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory("arm_vision")
CAMERA_NAME = "arm_cam"  # node name (topics /arm_cam/...) and camera_name (frames arm_cam_*)

declare_args = [
    DeclareLaunchArgument("camera", default_value="true",
                          description="Start realsense2_camera (the D435i driver)."),
    DeclareLaunchArgument("aruco", default_value="true",
                          description="Start the ArUco node on the camera's colour + depth."),
    DeclareLaunchArgument("video", default_value="true",
                          description="H.264 of every colour frame on /arm_cam/color/video "
                          "(camera_stream image_video_publisher: the Jetson's hardware encoder, "
                          "else x264): the smooth feed Foxglove shows, ArUco and keyboard drawn "
                          "over it."),
    DeclareLaunchArgument("video_bitrate", default_value="1000000",
                          description="H.264 bitrate cap, bits/s (1 Mbit/s: the radio link)."),
    DeclareLaunchArgument("video_encoder", default_value="auto", choices=["auto", "nvenc", "x264"],
                          description="auto = the hardware encoder if the Jetson has one (Orin "
                          "AGX / NX), else x264 in software (Orin Nano)."),
    DeclareLaunchArgument("standalone", default_value="true",
                          description="Also start foxglove_bridge on :8765 (false inside "
                          "rover.launch.py, which already runs it)."),
    DeclareLaunchArgument("serial_no", default_value="",
                          description="Open the RealSense with this serial number (empty = the "
                          "first one found). `rs-enumerate-devices -s` lists them."),
    DeclareLaunchArgument("realsense_config",
                          default_value=os.path.join(SHARE, "config", "realsense_d435i.yaml"),
                          description="realsense2_camera parameter file."),
    DeclareLaunchArgument("aruco_config",
                          default_value=os.path.join(SHARE, "config", "aruco.yaml"),
                          description="aruco_node parameter file."),
    DeclareLaunchArgument("dictionary", default_value="",
                          description="ArUco dictionary, e.g. DICT_5X5_50 "
                          "(empty = aruco_config)."),
    DeclareLaunchArgument("marker_size", default_value="",
                          description="Printed marker side in metres, black square only "
                          "(empty = aruco_config)."),
    DeclareLaunchArgument("marker_sizes", default_value="",
                          description="Markers printed at another size, 'id:size_m' or "
                          "'first-last:size_m', separated by commas, e.g. 10-13:0.04. Merged "
                          "over aruco_config's marker_sizes, and given to keyboard_node too (the "
                          "keyboard's markers). Empty = the config files'."),
    DeclareLaunchArgument("keyboard", default_value="true",
                          description="Start keyboard_node: the keyboard's pose and keys from "
                          "the ArUco markers on it (config/keyboard_layout.yaml)."),
    DeclareLaunchArgument("keyboard_config",
                          default_value=os.path.join(SHARE, "config", "keyboard.yaml"),
                          description="keyboard_node parameter file."),
    DeclareLaunchArgument("keyboard_layout", default_value="",
                          description="Keyboard layout YAML (empty = keyboard_config's, i.e. "
                          "config/keyboard_layout.yaml)."),
    DeclareLaunchArgument("target_frame", default_value="base_link",
                          description="Frame keyboard_node reports the keyboard in (base_link: "
                          "relative to the rover, through the arm's TF). Empty = the camera's "
                          "optical frame."),
    DeclareLaunchArgument("arm_tf", default_value="auto",
                          choices=["auto", "arm", "zero", "gui", "false"],
                          description="Start the arm's robot_state_publisher (arm_description) "
                          "so the camera is placed on the rover: arm = joint angles from "
                          "arm.launch.py (/arm/joint_states); zero = every joint held at 0 (bench, "
                          "arm not running); gui = sliders; auto = arm if arm_description is "
                          "built; false = none (rover.launch.py starts it itself)."),
    DeclareLaunchArgument("parent_frame", default_value="",
                          description="No arm URDF: publish a static TF parent_frame -> "
                          "arm_cam_link with the mount_* offsets instead, and skip arm_tf. Empty "
                          "= none: the arm's URDF (arm_tf) attaches arm_cam_link."),
    *[DeclareLaunchArgument(f"mount_{k}", default_value="0.0",
                            description=f"parent_frame:= only. Camera mount {k} "
                            f"({'m' if k in 'xyz' else 'rad'}).")
      for k in ("x", "y", "z", "roll", "pitch", "yaw")],
    DeclareLaunchArgument("log_level", default_value="info",
                          description="realsense2_camera log level (debug for USB problems)."),
    DeclareLaunchArgument("use_sim_time", default_value="false",
                          description="true with a sim (rover.launch.py mode:=sim): the ArUco "
                          "and keyboard nodes and the arm's TF run on /clock, and the ArUco "
                          "node restamps its outputs, since the camera itself stays on the "
                          "wall clock."),
]


def _truthy(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


def config_marker_sizes(path) -> dict[int, float]:
    """marker_sizes from an aruco_node parameter file ({} if it has none)."""
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    for node in data.values():
        params = (node or {}).get("ros__parameters", {}) if isinstance(node, dict) else {}
        if "marker_sizes" in params:
            return parse_marker_sizes(params["marker_sizes"])
    return {}


def _setup(context):
    actions = []
    sim_time = _truthy(context, "use_sim_time")
    sizes_arg = LaunchConfiguration("marker_sizes").perform(context)
    try:
        size_override = parse_marker_sizes(sizes_arg)
    except ValueError as e:
        raise RuntimeError(f"arm_camera.launch.py marker_sizes:={sizes_arg}: {e}") from None

    if _truthy(context, "camera"):
        cameras = realsense_cameras()
        if cameras:
            actions += [LogInfo(msg=f"arm_camera.launch.py: {c.describe()}") for c in cameras]
            actions += [LogInfo(msg=f"arm_camera.launch.py: WARNING {w}")
                        for w in camera_warnings(cameras)]
        else:
            actions.append(LogInfo(msg=(
                "arm_camera.launch.py: no RealSense on USB yet; realsense2_camera waits for one. "
                "Check `lsusb | grep 8086` and the cable (USB 3).")))
        # Always the wall clock: the RealSense is hardware and stamps its frames with the system
        # time. With use_sim_time:=true the ArUco node restamps its outputs on the sim clock.
        overrides = {"camera_name": CAMERA_NAME, "use_sim_time": False}
        serial = LaunchConfiguration("serial_no").perform(context)
        if serial:
            # Quoted, as realsense2_camera's own rs_launch.py does: the value goes through a YAML
            # file, where a serial number made of digits would turn into a number, and the
            # driver then dies with "parameter 'serial_no' has invalid type".
            overrides["serial_no"] = f"'{serial.strip(chr(39))}'"
        actions.append(Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name=CAMERA_NAME,
            namespace="",
            output="screen",
            emulate_tty=True,
            # The driver reconnects by itself after a USB drop; this covers a crash.
            respawn=True,
            respawn_delay=3.0,
            parameters=[LaunchConfiguration("realsense_config").perform(context), overrides],
            arguments=["--ros-args", "--log-level",
                       LaunchConfiguration("log_level").perform(context)],
        ))

    if _truthy(context, "video"):
        actions.append(Node(
            package="camera_stream",
            executable="image_video_publisher",
            name="color_video",
            namespace=CAMERA_NAME,
            output="screen",
            respawn=True,
            respawn_delay=3.0,
            parameters=[{
                "image_topic": "color/image_raw",
                "video_topic": "color/video",
                "encoder": LaunchConfiguration("video_encoder").perform(context),
                "bitrate": int(LaunchConfiguration("video_bitrate").perform(context)),
                "fps": 30,
                "gop": 30,
                # Like the ArUco node: on the sim clock with use_sim_time, so Foxglove (on
                # Gazebo's time) can draw the keyboard over the video through TF.
                "use_sim_time": sim_time,
                "restamp": sim_time,
            }],
        ))

    if _truthy(context, "aruco"):
        # Sim time: outputs restamped on the sim clock, so markers, the keyboard and the arm's
        # TF (arm_description, also on sim time) share the clock of the rover's TF and viewers.
        overrides = {"use_sim_time": sim_time, "restamp": sim_time}
        dictionary = LaunchConfiguration("dictionary").perform(context)
        if dictionary:
            overrides["dictionary"] = dictionary
        size = LaunchConfiguration("marker_size").perform(context)
        if size:
            overrides["marker_size"] = float(size)
        config = LaunchConfiguration("aruco_config").perform(context)
        if size_override:
            # Merged over the file's list: ids not named keep the size the config gives them.
            overrides["marker_sizes"] = format_marker_sizes(
                {**config_marker_sizes(config), **size_override})
        actions.append(Node(
            package="arm_vision",
            executable="aruco_node",
            name="aruco_node",
            namespace=CAMERA_NAME,
            output="screen",
            respawn=True,
            respawn_delay=3.0,
            parameters=[config, overrides],
        ))

    if _truthy(context, "keyboard"):
        overrides = {"use_sim_time": sim_time,
                     "target_frame": LaunchConfiguration("target_frame").perform(context)}
        layout = LaunchConfiguration("keyboard_layout").perform(context)
        if layout:
            overrides["layout_file"] = layout
        if size_override:
            overrides["marker_sizes"] = format_marker_sizes(size_override)
        actions.append(Node(
            package="arm_vision",
            executable="keyboard_node",
            name="keyboard_node",
            namespace=CAMERA_NAME,
            output="screen",
            respawn=True,
            respawn_delay=3.0,
            parameters=[LaunchConfiguration("keyboard_config").perform(context), overrides],
        ))

    parent = LaunchConfiguration("parent_frame").perform(context)
    actions += arm_tf_actions(LaunchConfiguration("arm_tf").perform(context), parent, sim_time)
    if parent:
        mount = []
        for k in ("x", "y", "z", "roll", "pitch", "yaw"):
            mount += [f"--{k}", LaunchConfiguration(f"mount_{k}").perform(context)]
        actions.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="arm_cam_mount_tf",
            output="screen",
            arguments=mount + ["--frame-id", parent, "--child-frame-id", f"{CAMERA_NAME}_link"],
            parameters=[{"use_sim_time": sim_time}],
        ))

    if _truthy(context, "standalone"):
        actions.append(Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[{
                "port": 8765,
                "address": "0.0.0.0",
                "topic_whitelist": [".*"],
                "send_buffer_limit": 2000000,  # bytes queued per client before dropping: 10 MB held ~1 s of lag on a slow link
                "use_sim_time": sim_time,
            }],
        ))
    return actions


def arm_tf_actions(mode, parent_frame="", sim_time=False):
    """arm_description.launch.py (the arm's robot_state_publisher), or why it's skipped."""
    if mode == "false":
        return []
    if parent_frame:
        return [LogInfo(msg=(f"arm_camera.launch.py: parent_frame:={parent_frame} mounts the "
                             "camera, so the arm's TF (arm_tf) is not started."))]
    try:
        share = get_package_share_directory("arm_description")
    except PackageNotFoundError:
        if mode == "auto":
            return [LogInfo(msg=(
                "arm_camera.launch.py: arm_description isn't built, so nothing places the "
                "camera on the rover (pixi run build --packages-up-to arm_description)."))]
        raise
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch",
                                                   "arm_description.launch.py")),
        launch_arguments={"joint_states": "arm" if mode == "auto" else mode,
                          "with_camera": "true", "rviz": "false",
                          "use_sim_time": "true" if sim_time else "false"}.items(),
    )]


def generate_launch_description():
    return LaunchDescription(declare_args + [OpaqueFunction(function=_setup)])
