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
    common = {"foxglove": "true", "rviz": LaunchConfiguration("rviz")}

    if mode == "sim":
        target = "sim_gz.launch.py"
        args = common
    elif mode == "real":
        target = "real.launch.py"
        args = {
            **common,
            "can_interface": LaunchConfiguration("can_interface"),
            "gear_ratio": LaunchConfiguration("gear_ratio"),
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
