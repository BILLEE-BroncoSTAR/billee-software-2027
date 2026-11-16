"""The arm's Intel RealSense D435i: realsense2_camera + the ArUco node (+ Foxglove bridge).

    # on its own (Jetson, l4t container): camera + ArUco + foxglove_bridge on :8765
    ros2 launch arm_vision arm_camera.launch.py

    # inside rover.launch.py (arm_camera:=auto|true): the drivetrain already runs the bridge
    ros2 launch arm_vision arm_camera.launch.py standalone:=false

    # markers: dictionary and printed size (m) of the black square
    ros2 launch arm_vision arm_camera.launch.py dictionary:=DICT_4X4_50 marker_size:=0.1

    # camera only / ArUco only (e.g. against a rosbag of /arm_cam/*)
    ros2 launch arm_vision arm_camera.launch.py aruco:=false
    ros2 launch arm_vision arm_camera.launch.py camera:=false

    # until the arm's URDF has the camera: where it sits on the end effector (m, rad)
    ros2 launch arm_vision arm_camera.launch.py parent_frame:=end_effector mount_z:=0.04

Topics (/arm_cam = the realsense2_camera node; config/realsense_d435i.yaml)
  /arm_cam/color/image_raw (+ /compressed)               colour, 640x480 @ 30, rgb8
  /arm_cam/color/camera_info
  /arm_cam/aligned_depth_to_color/image_raw (+ /compressedDepth)   depth on the colour pixels, mm
  /arm_cam/depth/image_rect_raw                          raw depth, 848x480
  /arm_cam/imu                                           gyro + accel (D435i)
  /arm_cam/aruco/markers                                 arm_vision_msgs/ArucoMarkers
  /arm_cam/aruco/image/compressed                        annotated JPEG: the ground station's feed
  /arm_cam/aruco/visualization                           MarkerArray for 3D panels
TF: arm_cam_link -> arm_cam_{color,depth,...}_optical_frame (driver), optical frame -> aruco_<id>.

realsense2_camera waits for the camera and reconnects after a USB drop, so the launch can start
before the camera is plugged in. Foxglove layout: arm_vision/foxglove/arm_camera.json.
"""

import os

from ament_index_python.packages import get_package_share_directory
from arm_vision.usb import camera_warnings, realsense_cameras
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory("arm_vision")
CAMERA_NAME = "arm_cam"  # node name (topics /arm_cam/...) and camera_name (frames arm_cam_*)

declare_args = [
    DeclareLaunchArgument("camera", default_value="true",
                          description="Start realsense2_camera (the D435i driver)."),
    DeclareLaunchArgument("aruco", default_value="true",
                          description="Start the ArUco node on the camera's colour + depth."),
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
    DeclareLaunchArgument("parent_frame", default_value="",
                          description="Publish a static TF parent_frame -> arm_cam_link with the "
                          "mount_* offsets (e.g. the end effector link). Empty = none: the arm's "
                          "URDF attaches arm_cam_link instead."),
    *[DeclareLaunchArgument(f"mount_{k}", default_value="0.0",
                            description=f"parent_frame:= only. Camera mount {k} "
                            f"({'m' if k in 'xyz' else 'rad'}).")
      for k in ("x", "y", "z", "roll", "pitch", "yaw")],
    DeclareLaunchArgument("log_level", default_value="info",
                          description="realsense2_camera log level (debug for USB problems)."),
    DeclareLaunchArgument("use_sim_time", default_value="false"),
]


def _truthy(context, name):
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


def _setup(context):
    actions = []
    sim_time = _truthy(context, "use_sim_time")

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
        overrides = {"camera_name": CAMERA_NAME, "use_sim_time": sim_time}
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

    if _truthy(context, "aruco"):
        overrides = {"use_sim_time": sim_time}
        dictionary = LaunchConfiguration("dictionary").perform(context)
        if dictionary:
            overrides["dictionary"] = dictionary
        size = LaunchConfiguration("marker_size").perform(context)
        if size:
            overrides["marker_size"] = float(size)
        actions.append(Node(
            package="arm_vision",
            executable="aruco_node",
            name="aruco_node",
            namespace=CAMERA_NAME,
            output="screen",
            respawn=True,
            respawn_delay=3.0,
            parameters=[LaunchConfiguration("aruco_config").perform(context), overrides],
        ))

    parent = LaunchConfiguration("parent_frame").perform(context)
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
                "send_buffer_limit": 10000000,
                "use_sim_time": sim_time,
            }],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription(declare_args + [OpaqueFunction(function=_setup)])
