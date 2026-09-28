"""One-command rover bring-up.

Wraps the existing backends and always starts the foxglove bridge:

    ros2 launch chassis_bringup rover.launch.py                       # sim, no ODESC
    ros2 launch chassis_bringup rover.launch.py mode:=real            # ODESC over CAN
    ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=mock

`mode:=sim`  -> sim_gz.launch.py  (Gazebo + ign_ros2_control, no CAN)
`mode:=real` -> real.launch.py    (standalone controller_manager + OdescSystemHardware)

The rover is headless, so `rviz` defaults false; the foxglove bridge (:8765) is always on.
The ground station runs its own view with `ground_station.launch.py`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
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
        "localization",
        default_value="true",
        description="Start the robot_localization EKF, which owns the odom -> base_link "
        "transform. Off means nothing publishes it and the TF tree is broken.",
    ),
    DeclareLaunchArgument(
        "joy_source",
        default_value="device",
        choices=["device", "browser"],
        description="Where /joy comes from. 'device' = a pad on /dev/input via joy_node "
        "(needs the `joy` package, which the ROVER environment does not ship). "
        "'browser' = the pad is plugged into the machine running Foxglove and its "
        "Joystick panel publishes /joy over the bridge - no joy_node, and the only way "
        "to drive the real rover from a Mac.",
    ),
    DeclareLaunchArgument(
        "joy_control",
        default_value="",
        description="Start gamepad teleop (joy_node + joy_drive) alongside, publishing "
        "to /diff_drive_controller/cmd_vel_unstamped. Left empty the default follows "
        "mode: true for sim (a ground station, which has the pad and the joy package) "
        "and false for real (the rover, which has neither - drive it from a ground "
        "station over DDS). Pass true/false to force it.",
    ),
    DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="mode:=real only. 'mock'/'none' = no CAN (loopback feedback); "
        "'vcan0' = virtual bus.",
    ),
    DeclareLaunchArgument(
        "gear_ratio",
        default_value="48.0",
        description="mode:=real only. Motor-shaft turns per wheel turn.",
    ),
]


def _setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    common = {
        "foxglove": "true",
        "rviz": LaunchConfiguration("rviz"),
        "localization": LaunchConfiguration("localization"),
    }

    # The operator's pad lives on the ground station, so the rover environment ships
    # no `joy` package at all (ros2_ws/pixi.toml, feature.rover). Starting teleop on
    # mode:=real would abort the whole launch with "package 'joy' not found", so the
    # default is off there and on for the sim, which runs on a ground station.
    joy_control = LaunchConfiguration("joy_control").perform(context)
    if joy_control == "":
        joy_control = "true" if mode == "sim" else "false"

    if mode == "sim":
        target = "sim_gz.launch.py"
        args = {
            **common,
            "joy_control": joy_control,
            "joy_source": LaunchConfiguration("joy_source"),
        }
    elif mode == "real":
        target = "real.launch.py"
        args = {
            **common,
            "can_interface": LaunchConfiguration("can_interface"),
            "gear_ratio": LaunchConfiguration("gear_ratio"),
            "joy_control": joy_control,
            "joy_source": LaunchConfiguration("joy_source"),
        }
    else:  # DeclareLaunchArgument(choices=...) already guards this
        raise RuntimeError(f"rover.launch.py: unknown mode {mode!r} (use sim|real)")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("chassis_bringup"), "launch", target]
                )
            ),
            launch_arguments=args.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(declare_args + [OpaqueFunction(function=_setup)])
