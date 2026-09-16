"""Shadow the Gazebo drive command through OdescSystemHardware on virtual CAN.

This is a Nav2 integration test, not a second physics simulator: Gazebo drives
the visible rover while this isolated controller manager receives the exact same
Twist, emits ODrive CANSimple commands, and consumes simulated encoder frames.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _controllers_file(context):
    src = os.path.join(
        get_package_share_directory("robot_description"), "config", "controllers.yaml"
    )
    with open(src) as file_handle:
        config = yaml.safe_load(file_handle)
    config.setdefault("controller_manager", {}).setdefault("ros__parameters", {})[
        "use_sim_time"
    ] = False
    # The shadow must not publish the Gazebo rover's odom -> base_link TF. Its
    # namespaced odometry remains available for comparison, but Gazebo is the
    # single authoritative TF publisher in this integration test.
    shadow_diff_drive = config.setdefault("diff_drive_controller", {}).setdefault(
        "ros__parameters", {}
    )
    shadow_diff_drive["enable_odom_tf"] = False
    shadow_diff_drive["odom_frame_id"] = "odesc_shadow_odom"
    shadow_diff_drive["base_frame_id"] = "odesc_shadow_base_link"

    # ROS parameter YAML keys are node names.  The ordinary controller file is
    # rooted at /controller_manager, but this test manager lives at
    # /odesc_shadow/controller_manager; wrap every controller node in that
    # namespace so controller_manager sees diff_drive_controller.type.
    config = {"odesc_shadow": config}
    fd, path = tempfile.mkstemp(prefix="controllers_odesc_shadow_", suffix=".yaml")
    with os.fdopen(fd, "w") as file_handle:
        yaml.safe_dump(config, file_handle, default_flow_style=False)
    return path


def _launch_setup(context):
    description_pkg = LaunchConfiguration("description_pkg")
    xacro_file = LaunchConfiguration("xacro_file")
    can_interface = LaunchConfiguration("can_interface")
    xacro_path = PathJoinSubstitution([FindPackageShare(description_pkg), xacro_file])
    robot_description = ParameterValue(
        Command([
            "xacro", " ", xacro_path,
            " ", "use_sim:=false",
            " ", "can_interface:=", can_interface,
        ]),
        value_type=str,
    )

    # A controller-manager namespace isolates its /odom, TF and joint-state
    # output from the Gazebo manager. The one input is deliberately remapped
    # back to the Gazebo controller's public command topic.
    shadow_command_topic = "/odesc_shadow/diff_drive_controller/cmd_vel_unstamped"
    gazebo_command_topic = "/diff_drive_controller/cmd_vel_unstamped"
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="odesc_shadow",
        output="screen",
        remappings=[(shadow_command_topic, gazebo_command_topic)],
        parameters=[{"robot_description": robot_description}, _controllers_file(context)],
    )
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller", "joint_state_broadcaster",
            "--controller-manager", "/odesc_shadow/controller_manager",
        ],
        output="screen",
    )
    emulator = ExecuteProcess(
        cmd=[
            os.path.join(get_package_prefix("odesc"), "lib", "odesc", "odesc_vcan_emulator.py"),
            "--interface", can_interface,
        ],
        output="screen",
    )
    return [emulator, controller_manager, controller_spawner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("description_pkg", default_value="robot_description"),
        DeclareLaunchArgument("xacro_file", default_value="urdf/robot.urdf.xacro"),
        DeclareLaunchArgument("can_interface", default_value="vcan0"),
        OpaqueFunction(function=_launch_setup),
    ])
