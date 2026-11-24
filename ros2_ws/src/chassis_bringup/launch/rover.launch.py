"""One-command rover bring-up.

Wraps the existing backends and always starts the foxglove bridge:

    ros2 launch chassis_bringup rover.launch.py                       # sim, no ODESC
    ros2 launch chassis_bringup rover.launch.py mode:=real            # ODESC over CAN
    ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=vcan0  # emulator, no motors
    ros2 launch chassis_bringup rover.launch.py mode:=real camera:=video     # + ZED video
    ros2 launch chassis_bringup rover.launch.py mode:=real camera:=cloud     # + video + point cloud
    ros2 launch chassis_bringup rover.launch.py mode:=real camera:=openpose  # + video + OpenPose
    ros2 launch chassis_bringup rover.launch.py mode:=real camera:=both      # + video + cloud + OpenPose
    ros2 launch chassis_bringup rover.launch.py mode:=real vio:=true         # + ZED VIO + EKF
    ros2 launch chassis_bringup rover.launch.py mode:=real arm_camera:=false # no arm RealSense
    ros2 launch chassis_bringup rover.launch.py mode:=real arm_tf:=zero      # arm TF, arm off
    ros2 launch chassis_bringup rover.launch.py marker_sizes:=10-13:0.04     # keyboard markers 40 mm
    ros2 launch chassis_bringup rover.launch.py mode:=real status_light:=teleop  # blink(1): blue

`mode:=sim`  -> sim_gz.launch.py  (Gazebo + ign_ros2_control, no CAN)
`mode:=real` -> real.launch.py    (standalone controller_manager + OdescSystemHardware)

The rover is headless, so `rviz` defaults false; the foxglove bridge (:8765) is always on.
The ground station runs its own view and the gamepad teleop with `ground_station.launch.py`,
so `joy_control` defaults false here: a second joy_tank_drive on the rover would also
react to the ground station's /joy and publish every drive command twice.

`vio:=true` (mode:=real) is the real counterpart of the sim's pose estimator. The ZED publishes
its visual-inertial odometry (/zed/vio/odom), and a robot_localization EKF
(navigation/config/ekf_real.yaml) fuses it with the wheel odometry and publishes
`odom -> base_footprint`. The wheel-odometry TF is then off (real.launch.py odom_tf:=false),
since a TF frame can have only one publisher. It combines with any camera:=.

The CSI cameras (camera_stream/launch/csi_cameras.launch.py, every camera in
camera_stream/config/cameras.yaml) always start, whatever the arguments, since they are real
hardware on the Jetson even when the drivetrain is simulated: /camN/video (H.264, Foxglove) and
/camN/image_raw/compressed (JPEG, RViz). There is no argument to turn them off. A camera that
isn't connected publishes a "NO CAMERA" placeholder and never stops the launch. In mode:=real, a
missing or down CAN interface skips the drivetrain with an error instead of stopping the launch,
so the cameras (and the foxglove bridge) keep running. Skipped only where there are no NVIDIA
camera plugins (any machine but the Jetson).

The arm's RealSense D435i (arm_vision/launch/arm_camera.launch.py) also starts in both modes:
arm_camera:=auto (default) starts realsense2_camera + the ArUco node when a RealSense is on USB at
launch, true starts them regardless (the driver waits for the camera), false never. The ground
station's view of it: /arm_cam/aruco/image/compressed (annotated JPEG) and
/arm_cam/color/image_raw/compressed, in Foxglove (chassis_bringup/foxglove/rover_arm.json, the
Teleop · Arm layout) or RViz. With it comes keyboard_node: the keyboard's pose and keys (/arm_cam/keyboard/*, TF
base_link -> keyboard) from the ArUco markers on it.

The arm's TF (arm_description/launch/arm_description.launch.py, arm_tf:=auto) places the arm and
the RealSense on the rover: base_link -> arm_base_link -> ... -> arm_cam_link from the joint
angles on /arm/joint_states (arm_bringup's arm.launch.py, started separately). auto = arm in
mode:=real, zero (every joint at 0 rad) in mode:=sim; pass arm_tf:=arm in mode:=sim when
arm.launch.py runs (real or mock arm), or the two publish /arm/joint_states over each other.
In mode:=sim the arm's TF, the ArUco and the keyboard nodes run on the sim clock with the rest
of the TF tree, so Foxglove and RViz place the arm, the RealSense on it and the markers in odom.
The RealSense driver and arm.launch.py stay on the wall clock (hardware); the ArUco node and
arm_description restamp what they get from them.
"""

import importlib.util
import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

declare_args = [
    DeclareLaunchArgument(
        "mode",
        default_value="sim",
        choices=["sim", "real"],
        description="sim = Gazebo, no ODESC (sim_gz.launch.py); "
        "real = ODESC over CAN (real.launch.py).",
    ),
    DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Also start RViz on the rover (needs a display / xvfb-run). "
        "Normally the ground station runs RViz.",
    ),
    DeclareLaunchArgument(
        "joy_control",
        default_value="false",
        description="Start gamepad teleop on the rover. Normally the ground station runs it; "
        "true only with no ground station (e.g. onboard_test.launch.py).",
    ),
    DeclareLaunchArgument(
        "gui",
        default_value="false",
        description="mode:=sim only. Open the Gazebo window. The rover has no monitor, so "
        "run under `xvfb-run -a vglrun -d egl` (GPU rendering, see docs/gazebo.md).",
    ),
    DeclareLaunchArgument(
        "can_interface",
        default_value="can2",
        description="mode:=real only. The drivetrain's SocketCAN bus. The launch stops and "
        "lists the CAN interfaces if it is missing or down. 'vcan0' = virtual bus "
        "(with odesc_vcan_emulator.py, no motors). 'mock'/'none' is currently broken.",
    ),
    DeclareLaunchArgument(
        "gear_ratio",
        default_value="48.0",
        description="mode:=real only. Motor-shaft turns per wheel turn.",
    ),
    DeclareLaunchArgument(
        "fault_policy",
        default_value="warn",
        choices=["warn", "fault"],
        description="mode:=real only. Unhealthy wheel: warn = log and keep driving; "
        "fault = stop the drivetrain.",
    ),
    DeclareLaunchArgument(
        "odom_tf",
        default_value="true",
        description="mode:=real only. Publish odom -> base_footprint TF from the wheel "
        "odometry so the ground station's fixed frame odom exists. false once an EKF on the "
        "rover publishes it. vio:=true forces it false. (mode:=sim always uses the sim EKF.)",
    ),
    DeclareLaunchArgument(
        "camera",
        default_value="none",
        choices=["none", "video", "cloud", "openpose", "both"],
        description="ZED 2i on the Jetson. Every mode but none has the video "
        "(/zed/image/compressed): video = video only; cloud = video + point cloud (/zed/points); "
        "openpose = video + OpenPose people/gestures (/people/*), the video then shows the "
        "skeletons; both = video + point cloud + OpenPose. Jetson (l4t) only.",
    ),
    DeclareLaunchArgument(
        "vio",
        default_value="false",
        description="mode:=real only. ZED visual-inertial odometry (/zed/vio/odom) + an EKF "
        "that fuses it with the wheel odometry and publishes odom -> base_footprint "
        "(odom_tf is then false). Any camera:=. Jetson (l4t) only.",
    ),
    DeclareLaunchArgument(
        "arm_camera",
        default_value="auto",
        choices=["auto", "true", "false"],
        description="The arm's RealSense D435i + ArUco detection (/arm_cam/*): auto = if a "
        "RealSense is on USB at launch; true = always (waits for the camera); false = never. "
        "Both modes.",
    ),
    DeclareLaunchArgument(
        "marker_size",
        default_value="",
        description="Arm camera: printed ArUco marker side in metres, black square only "
        "(empty = arm_vision/config/aruco.yaml, 0.05).",
    ),
    DeclareLaunchArgument(
        "marker_sizes",
        default_value="",
        description="Arm camera: markers printed at another size, 'id:size_m' or "
        "'first-last:size_m' separated by commas, e.g. 10-13:0.04 (the keyboard's markers). "
        "The ArUco and keyboard nodes both use them. Empty = the config files' (keyboard 30 mm).",
    ),
    DeclareLaunchArgument(
        "arm_tf",
        default_value="auto",
        choices=["auto", "arm", "zero", "gui", "false"],
        description="The arm's robot_state_publisher (arm_description): base_link -> arm links "
        "-> arm_cam_link. arm = joint angles from arm.launch.py (/arm/joint_states); zero = all "
        "joints at 0 rad (arm not running); gui = sliders; auto = arm (real) / zero (sim); "
        "false = none.",
    ),
    DeclareLaunchArgument(
        "status_light",
        default_value="none",
        choices=["none", "idle", "teleop", "autonomous", "arrived", "off"],
        description="The blink(1) mode light (status_light package): teleop = solid blue, "
        "autonomous = solid red, arrived = flashing green, idle = slow yellow pulse. none = don't "
        "start it. The BILLEE dashboard sets it per mode. Both modes.",
    ),
]

# nvarguscamerasrc, present only in the Jetson (l4t) container.
CSI_PLUGIN = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnvarguscamerasrc.so"

# camera:= -> people_rover.launch.py (openpose, point_cloud, video). "none" runs the ZED for
# vio:= alone; every other mode has the video.
CAMERA_MODES = {
    "none": ("false", "false", "false"),
    "video": ("false", "false", "true"),
    "cloud": ("false", "true", "true"),
    "openpose": ("true", "false", "true"),
    "both": ("true", "true", "true"),
}


def _setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    camera, vio, actions = _camera_and_vio(context, mode)
    common = {"foxglove": "true", "rviz": LaunchConfiguration("rviz")}

    if mode == "sim":
        target = "sim_gz.launch.py"
        args = {
            **common,
            "gui": LaunchConfiguration("gui"),
            "joy_control": LaunchConfiguration("joy_control"),
        }
    elif mode == "real":
        can_problem = _can_problem(context)
        if can_problem:
            # real.launch.py would raise and take the whole launch down, cameras included.
            # Start everything but the drivetrain, plus the bridge real.launch.py would start.
            actions.append(LogInfo(msg=(
                f"rover.launch.py: ERROR, drivetrain NOT started: {can_problem} "
                "The cameras keep running; fix CAN and relaunch to drive.")))
            target = "viz.launch.py"
            args = {**common, "use_sim_time": "false"}
        else:
            target = "real.launch.py"
            args = {
                **common,
                "can_interface": LaunchConfiguration("can_interface"),
                "gear_ratio": LaunchConfiguration("gear_ratio"),
                "fault_policy": LaunchConfiguration("fault_policy"),
                # The EKF owns odom -> base_footprint; diff_drive_controller mustn't also send it.
                "odom_tf": "false" if vio else LaunchConfiguration("odom_tf"),
            }
        # real.launch.py has no teleop of its own (sim_gz does, via joy_control).
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("teleop"), "launch", "teleop.launch.py"]
                    )
                ),
                condition=IfCondition(LaunchConfiguration("joy_control")),
            )
        )
    else:  # DeclareLaunchArgument(choices=...) already guards this
        raise RuntimeError(f"rover.launch.py: unknown mode {mode!r} (use sim|real)")

    if camera != "none" or vio:
        # The drivetrain launch above already runs robot_state_publisher and the foxglove
        # bridge, so the camera launch runs without its own (standalone:=false).
        openpose, point_cloud, video = CAMERA_MODES[camera]
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("people"), "launch", "people_rover.launch.py"]
                    )
                ),
                launch_arguments={
                    "standalone": "false",
                    "openpose": openpose,
                    "point_cloud": point_cloud,
                    "video": video,
                    "vio": "true" if vio else "false",
                    "use_sim_time": "true" if mode == "sim" else "false",
                }.items(),
            )
        )
    actions = _csi_cameras(context, mode) + actions
    actions += _arm_tf(context, mode)
    actions += _arm_camera(context, mode)
    actions += _status_light(context)
    if vio:
        actions.append(
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("navigation"), "config", "ekf_real.yaml"]
                    ),
                    {"use_sim_time": False},
                ],
            )
        )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("chassis_bringup"), "launch", target]
                )
            ),
            launch_arguments=args.items(),
        ),
        *actions,
    ]


def _camera_and_vio(context, mode):
    """Resolve camera:= and vio:= to what can actually run here, logging what is dropped."""
    camera = LaunchConfiguration("camera").perform(context)
    vio = LaunchConfiguration("vio").perform(context).lower() in ("true", "1")
    logs = []
    if vio and mode != "real":
        logs.append(LogInfo(msg=(
            "rover.launch.py: vio:=true ignored in mode:=sim, whose EKF already fuses the "
            "synthetic VIO (navigation/pose_estim.launch.py).")))
        vio = False
    if (camera != "none" or vio) and not _camera_node_built():
        # zed_openpose only builds its node where the ZED SDK and OpenPose are installed (the
        # Jetson l4t image). Without it, keep the drivetrain running rather than abort the launch.
        wanted = " and ".join(
            ([f"camera:={camera}"] if camera != "none" else []) + (["vio:=true"] if vio else []))
        logs.append(LogInfo(msg=(
            f"rover.launch.py: {wanted} ignored, zed_openpose_node isn't built here. "
            "Build it in the Jetson (l4t) container: "
            "pixi run build --packages-select zed_openpose people")))
        camera, vio = "none", False
    if vio:
        logs.append(LogInfo(msg=(
            "rover.launch.py: vio:=true, the EKF publishes odom -> base_footprint "
            "(wheel-odometry TF off, odom_tf:=false).")))
    return camera, vio, logs


def _csi_cameras(context, mode):
    """The CSI camera nodes, always (no argument turns them off), or why they can't run here."""
    logs = []
    if "csi_cameras" in context.launch_configurations:
        logs.append(LogInfo(msg=(
            "rover.launch.py: csi_cameras:= is no longer an argument and is ignored: the CSI "
            "cameras always start. Change the camera list in camera_stream/config/cameras.yaml.")))
    if not os.path.exists(CSI_PLUGIN):
        return logs + [LogInfo(msg=(
            "rover.launch.py: CSI cameras not started, no NVIDIA camera plugins here "
            "(the CSI cameras run in the Jetson container)."))]
    return logs + [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("camera_stream"), "launch", "csi_cameras.launch.py"])),
        # Only use_sim_time: nothing the caller passes can change which cameras start.
        launch_arguments={"use_sim_time": "true" if mode == "sim" else "false"}.items(),
    )]


def _status_light(context):
    """The blink(1) mode light (status_light:=<pattern>), or why it isn't started."""
    pattern = LaunchConfiguration("status_light").perform(context)
    if pattern == "none":
        return []
    try:
        get_package_prefix("status_light")
    except PackageNotFoundError:
        return [LogInfo(msg=(
            f"rover.launch.py: status_light:={pattern} ignored, the status_light package isn't "
            "built here (pixi run build --packages-select status_light)."))]
    return [Node(package="status_light", executable="status_light", name="status_light",
                 output="screen", parameters=[{"pattern": pattern}])]


def _can_problem(context):
    """Run real.launch.py's CAN check here: its error message, or None if CAN is fine."""
    path = os.path.join(
        get_package_share_directory("chassis_bringup"), "launch", "real.launch.py")
    spec = importlib.util.spec_from_file_location("chassis_bringup_real_launch", path)
    real = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(real)
    try:
        real._check_can_interface(LaunchConfiguration("can_interface").perform(context))
    except RuntimeError as e:
        return str(e)
    return None


def _arm_camera(context, mode):
    """arm_camera.launch.py (RealSense + ArUco) without its own bridge, or why it's skipped."""
    wanted = LaunchConfiguration("arm_camera").perform(context)
    if wanted == "false":
        return []
    try:
        get_package_prefix("arm_vision")
        get_package_prefix("realsense2_camera")
    except PackageNotFoundError as e:
        return [LogInfo(msg=(
            f"rover.launch.py: arm_camera:={wanted} ignored, package {e} isn't installed/built "
            "here (pixi install, then pixi run build --packages-up-to arm_vision)."))]
    if wanted == "auto":
        from arm_vision.usb import realsense_cameras

        if not realsense_cameras():
            return [LogInfo(msg=(
                "rover.launch.py: no RealSense on USB, arm camera not started "
                "(arm_camera:=true starts it anyway and waits for the camera)."))]
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_vision"), "launch", "arm_camera.launch.py"])),
        launch_arguments={
            # An included launch inherits the parent's arguments, and this file's camera:= is the
            # ZED mode (camera:=cloud). arm_camera.launch.py's camera:= means "start
            # realsense2_camera", so "cloud" silently switched the RealSense driver off. Set it.
            "camera": "true",
            "aruco": "true",
            "standalone": "false",  # the drivetrain launch already runs the foxglove bridge
            "arm_tf": "false",      # _arm_tf() starts it, camera or not
            # mode:=sim: the ArUco and keyboard nodes on the sim clock, like the rover's TF and
            # the viewers (Foxglove's time, RViz). The RealSense driver stays on the wall clock
            # (hardware); the ArUco node restamps its outputs, so the markers and the keyboard
            # can be placed in odom. Markers stamped on the wall clock never showed in Foxglove.
            "use_sim_time": "true" if mode == "sim" else "false",
            "marker_size": LaunchConfiguration("marker_size").perform(context),
            "marker_sizes": LaunchConfiguration("marker_sizes").perform(context),
        }.items(),
    )]


def _arm_tf(context, mode):
    """arm_description.launch.py (the arm's robot_state_publisher), or why it's skipped."""
    wanted = LaunchConfiguration("arm_tf").perform(context)
    if wanted == "false":
        return []
    try:
        share = get_package_share_directory("arm_description")
    except PackageNotFoundError:
        return [LogInfo(msg=(
            "rover.launch.py: arm_description isn't built, so the arm (and the RealSense on it) "
            "has no TF (pixi run build --packages-up-to arm_description)."))]
    if wanted == "auto":
        wanted = "zero" if mode == "sim" else "arm"
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "arm_description.launch.py")),
        launch_arguments={
            "joint_states": wanted,
            "rviz": "false",
            # The rover's clock: sim time in mode:=sim, so base_link -> arm -> arm_cam_link joins
            # odom -> base_link in TF lookups and viewers. arm.launch.py (real or mock) stays on
            # the wall clock; arm_description restamps its /arm/joint_states on the sim clock.
            "use_sim_time": "true" if mode == "sim" else "false",
        }.items(),
    )]


def _camera_node_built():
    try:
        prefix = get_package_prefix("zed_openpose")
    except PackageNotFoundError:
        return False
    return os.access(os.path.join(prefix, "lib", "zed_openpose", "zed_openpose_node"), os.X_OK)


def generate_launch_description():
    return LaunchDescription(declare_args + [OpaqueFunction(function=_setup)])
