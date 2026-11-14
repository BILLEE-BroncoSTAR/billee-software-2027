"""One-command rover bring-up.

Wraps the existing backends and always starts the foxglove bridge:

    ros2 launch chassis_bringup rover.launch.py                       # sim, no ODESC
    ros2 launch chassis_bringup rover.launch.py mode:=real            # ODESC over CAN
    ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=mock
    ros2 launch chassis_bringup rover.launch.py mode:=real camera:=cloud     # + ZED point cloud
    ros2 launch chassis_bringup rover.launch.py mode:=real camera:=openpose  # + OpenPose
    ros2 launch chassis_bringup rover.launch.py mode:=real camera:=both      # + both
    ros2 launch chassis_bringup rover.launch.py mode:=real vio:=true         # + ZED VIO + EKF

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
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
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
        "lists the CAN interfaces if it is missing or down. 'mock'/'none' = no CAN "
        "(loopback feedback); "
        "'vcan0' = virtual bus.",
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
        choices=["none", "cloud", "openpose", "both"],
        description="ZED 2i on the Jetson: none; cloud = point cloud (/zed/points); "
        "openpose = OpenPose people/gestures (/people/*); both. Jetson (l4t) only.",
    ),
    DeclareLaunchArgument(
        "vio",
        default_value="false",
        description="mode:=real only. ZED visual-inertial odometry (/zed/vio/odom) + an EKF "
        "that fuses it with the wheel odometry and publishes odom -> base_footprint "
        "(odom_tf is then false). Any camera:=. Jetson (l4t) only.",
    ),
]

# camera:= -> people_rover.launch.py (openpose, point_cloud). "none" runs the ZED for vio:= alone.
CAMERA_MODES = {
    "none": ("false", "false"),
    "cloud": ("false", "true"),
    "openpose": ("true", "false"),
    "both": ("true", "true"),
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
        target = "real.launch.py"
        args = {
            **common,
            "can_interface": LaunchConfiguration("can_interface"),
            "gear_ratio": LaunchConfiguration("gear_ratio"),
            "fault_policy": LaunchConfiguration("fault_policy"),
            # The EKF owns odom -> base_footprint; diff_drive_controller must not also send it.
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
        openpose, point_cloud = CAMERA_MODES[camera]
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
                    "vio": "true" if vio else "false",
                    "use_sim_time": "true" if mode == "sim" else "false",
                }.items(),
            )
        )
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


def _camera_node_built():
    try:
        prefix = get_package_prefix("zed_openpose")
    except PackageNotFoundError:
        return False
    return os.access(os.path.join(prefix, "lib", "zed_openpose", "zed_openpose_node"), os.X_OK)


def generate_launch_description():
    return LaunchDescription(declare_args + [OpaqueFunction(function=_setup)])
