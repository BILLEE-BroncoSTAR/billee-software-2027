"""One-command rover bring-up.

Wraps the existing backends and always starts the foxglove bridge:

    ros2 launch chassis_bringup rover.launch.py                       # sim, no ODESC
    ros2 launch chassis_bringup rover.launch.py mode:=real            # ODESC over CAN
    ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=mock
    ros2 launch chassis_bringup rover.launch.py mode:=real camera:=cloud     # + ZED point cloud
    ros2 launch chassis_bringup rover.launch.py mode:=real camera:=openpose  # + OpenPose
    ros2 launch chassis_bringup rover.launch.py mode:=real camera:=both      # + both

`mode:=sim`  -> sim_gz.launch.py  (Gazebo + ign_ros2_control, no CAN)
`mode:=real` -> real.launch.py    (standalone controller_manager + OdescSystemHardware)

The rover is headless, so `rviz` defaults false; the foxglove bridge (:8765) is always on.
The ground station runs its own view and the gamepad teleop with `ground_station.launch.py`,
so `joy_control` defaults false here: a second joy_tank_drive on the rover would also
react to the ground station's /joy and publish every drive command twice.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
        default_value="can3",
        description="mode:=real only. 'mock'/'none' = no CAN (loopback feedback); "
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
        "camera",
        default_value="none",
        choices=["none", "cloud", "openpose", "both"],
        description="ZED 2i on the Jetson: none; cloud = point cloud (/zed/points); "
        "openpose = OpenPose people/gestures (/people/*); both. Jetson (l4t) only.",
    ),
]

# camera:= -> people_rover.launch.py (openpose, point_cloud)
CAMERA_MODES = {
    "cloud": ("false", "true"),
    "openpose": ("true", "false"),
    "both": ("true", "true"),
}


def _setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    common = {"foxglove": "true", "rviz": LaunchConfiguration("rviz")}
    actions = []

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

    camera = LaunchConfiguration("camera").perform(context)
    if camera != "none" and not _camera_node_built():
        # zed_openpose only builds its node where the ZED SDK and OpenPose are installed (the
        # Jetson l4t image). Without it, keep the drivetrain running rather than abort the launch.
        actions.append(LogInfo(msg=(
            f"rover.launch.py: camera:={camera} ignored, zed_openpose_node isn't built here. "
            "Build it in the Jetson (l4t) container: "
            "pixi run build --packages-select zed_openpose people")))
        camera = "none"
    if camera != "none":
        # The drivetrain launch above already runs robot_state_publisher and the foxglove
        # bridge, so the camera launch runs without its own (standalone:=false).
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("people"), "launch", "people_rover.launch.py"]
                    )
                ),
                launch_arguments={
                    "standalone": "false",
                    "openpose": CAMERA_MODES[camera][0],
                    "point_cloud": CAMERA_MODES[camera][1],
                    "use_sim_time": "true" if mode == "sim" else "false",
                }.items(),
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


def _camera_node_built():
    try:
        prefix = get_package_prefix("zed_openpose")
    except PackageNotFoundError:
        return False
    return os.access(os.path.join(prefix, "lib", "zed_openpose", "zed_openpose_node"), os.X_OK)


def generate_launch_description():
    return LaunchDescription(declare_args + [OpaqueFunction(function=_setup)])
