"""Real-hardware drivetrain bring-up (no Gazebo).

Parallel to sim_gz.launch.py: same robot_description, same controllers.yaml, same
diff_drive_controller + joint_state_broadcaster. The only difference is the
ros2_control backend — this expands the URDF with use_sim:=false so the
OdescSystemHardware SocketCAN plugin is loaded instead of ign_ros2_control, and
it starts a standalone controller_manager (ros2_control_node) rather than letting
Gazebo host it.

    # real ODESC/NEO drivetrain on can0:
    ros2 launch chassis_bringup real.launch.py

    # no motor hardware — exercise the full feedback -> odom -> TF -> viewer path:
    ros2 launch chassis_bringup real.launch.py can_interface:=mock rviz:=true

    # protocol-level test against a virtual CAN bus (see tooling/can-up):
    ros2 launch chassis_bringup real.launch.py can_interface:=vcan0

WHY THIS FILE PATCHES use_sim_time
----------------------------------
`controllers.yaml` carries `use_sim_time: true` for the Gazebo path. A *standalone*
`ros2_control_node` (this launch) started with `use_sim_time:=true` but no `/clock`
publisher has a frozen RT update loop, and `load_controller` then blocks forever.
So this launch writes a temp copy of `controllers.yaml` with `use_sim_time: false`
and hands the controller_manager that. `controllers.yaml` itself is untouched.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

declare_args = [
    DeclareLaunchArgument(
        "description_pkg",
        default_value="robot_description",
        description="Package that contains the robot URDF/Xacro in its share/ directory.",
    ),
    DeclareLaunchArgument(
        "xacro_file",
        default_value="urdf/robot.urdf.xacro",
        description="Path to the robot Xacro (relative to the description package share).",
    ),
    DeclareLaunchArgument(
        "controllers_file",
        default_value="config/controllers.yaml",
        description="ros2_control controller config (relative to the description package "
        "share). Same file sim_gz.launch.py uses; this launch only flips use_sim_time.",
    ),
    DeclareLaunchArgument(
        "can_interface",
        default_value="can0",
        description="SocketCAN interface for the ODESC bus. 'mock' or 'none' runs "
        "OdescSystemHardware in mock mode (no CAN, gear-ratio loopback feedback) so the "
        "stack can be brought up with no motor hardware. 'vcan0' etc. for a virtual bus.",
    ),
    DeclareLaunchArgument(
        "gear_ratio",
        default_value="48.0",
        description="Motor-shaft turns per wheel turn. Default 48.0 (ODESC V4.2 + NEO "
        "REV v1.1). Canonical copy in odesc/config/node_map.yaml.",
    ),
    DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Also start RViz2 with the drivetrain view (needs a display; use "
        "xvfb-run for headless CI). See viz.launch.py.",
    ),
    DeclareLaunchArgument(
        "foxglove",
        default_value="false",
        description="Also start the foxglove_bridge WebSocket server on :8765.",
    ),
]


def _write_wallclock_controllers(context):
    """Copy the controllers YAML with controller_manager.use_sim_time forced false."""
    desc_pkg = LaunchConfiguration("description_pkg").perform(context)
    rel = LaunchConfiguration("controllers_file").perform(context)
    src = os.path.join(get_package_share_directory(desc_pkg), *rel.split("/"))
    with open(src) as fh:
        cfg = yaml.safe_load(fh)

    cm = cfg.setdefault("controller_manager", {}).setdefault("ros__parameters", {})
    cm["use_sim_time"] = False

    fd, path = tempfile.mkstemp(prefix="controllers_real_", suffix=".yaml")
    with os.fdopen(fd, "w") as fh:
        yaml.safe_dump(cfg, fh, default_flow_style=False)
    return path


def _launch_setup(context):
    description_pkg = LaunchConfiguration("description_pkg")
    xacro_file = LaunchConfiguration("xacro_file")
    can_interface = LaunchConfiguration("can_interface")
    gear_ratio = LaunchConfiguration("gear_ratio")

    xacro_path = PathJoinSubstitution([FindPackageShare(description_pkg), xacro_file])

    # use_sim:=false -> ros2_control.urdf.xacro loads odesc/OdescSystemHardware.
    # can_interface / gear_ratio are forwarded into the <hardware> block.
    robot_description = ParameterValue(
        Command(
            [
                "xacro", " ", xacro_path,
                " ", "use_sim:=false",
                " ", "can_interface:=", can_interface,
                " ", "gear_ratio:=", gear_ratio,
            ]
        ),
        value_type=str,
    )

    controllers_path = _write_wallclock_controllers(context)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )

    # Standalone controller_manager. robot_description is passed as a parameter
    # (works today; the "~/robot_description topic" path is not reliable with the
    # RT loop on this target). use_sim_time is already false in controllers_path.
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[{"robot_description": robot_description}, controllers_path],
    )

    # Same spawner invocation as sim_gz.launch.py.
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "joint_state_broadcaster"],
    )

    viz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("chassis_bringup"), "launch", "viz.launch.py"]
            )
        ),
        launch_arguments={
            "rviz": LaunchConfiguration("rviz"),
            "foxglove": LaunchConfiguration("foxglove"),
            "use_sim_time": "false",
        }.items(),
    )

    return [robot_state_publisher, controller_manager, controller_spawner, viz]


def generate_launch_description():
    return LaunchDescription(declare_args + [OpaqueFunction(function=_launch_setup)])
