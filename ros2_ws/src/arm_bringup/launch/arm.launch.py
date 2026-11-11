"""Bring up everything on the BILLEE arm: P17/P20 (CAN) and RoboClaw (UART) axes.

Everything is generated from config/arm_axes.yaml (the physical constants), so
adding, renaming or re-gearing an axis is a YAML edit only.

    # real hardware (arm CAN bus on can1, RoboClaws on the /dev/roboclaw UART):
    ros2 launch arm_bringup arm.launch.py

    # no hardware at all (pxx mock backend + mock RoboClaw):
    ros2 launch arm_bringup arm.launch.py can_interface:=mock roboclaw_port:=mock

    # CAN protocol test against the CANopen emulator (tooling/can-up vcan0 first):
    ros2 launch arm_bringup arm.launch.py can_interface:=vcan0 roboclaw_port:=mock

    # only one half of the arm:
    ros2 launch arm_bringup arm.launch.py roboclaw_port:=none   # CAN axes only
    ros2 launch arm_bringup arm.launch.py can_interface:=none   # RoboClaw axes only

What it starts:
  - ros2_control_node with up to two hardware systems:
      ArmCan       pxx/PxxSystemHardware                     (driver: pxx axes)
      ArmRoboClaw  roboclaw_hardware_interface/RoboClawHardwareInterface
                   (or mock_components/GenericSystem when roboclaw_port:=mock)
  - spawners: joint_state_broadcaster, arm_can_controller (JointTrajectoryController),
    arm_roboclaw_controller (JointGroupVelocityController)
  - pxx_vcan_emulator.py when can_interface is a vcanN interface

There is no arm kinematic model yet, so no robot_state_publisher / TF: the
generated URDF only carries the <ros2_control> blocks. /joint_states is published.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DISABLED = ("none", "off", "false", "")
ROBOCLAW_ADDRESSES = range(128, 136)


def _load_axes(path):
    with open(path) as f:
        axes = (yaml.safe_load(f) or {}).get("axes") or []
    if not axes:
        raise RuntimeError(f"{path}: no axes defined")

    seen_joints, seen_nodes, seen_channels = set(), set(), set()
    for axis in axes:
        name = axis.get("joint")
        driver = axis.get("driver")
        where = f"{path}: axis '{name}'"
        if not name or name in seen_joints:
            raise RuntimeError(f"{where}: missing or duplicate joint name")
        seen_joints.add(name)

        reduction = float(axis["builtin_reduction"]) * float(axis.get("external_reduction", 1.0))
        direction = int(axis.get("direction", 1))
        if reduction <= 0 or direction not in (1, -1):
            raise RuntimeError(f"{where}: reductions must be > 0 and direction +1/-1")

        if driver == "pxx":
            node_id = int(axis["node_id"])
            if not 1 <= node_id <= 127 or node_id in seen_nodes:
                raise RuntimeError(f"{where}: node_id must be unique and in 1-127")
            seen_nodes.add(node_id)
            axis["gear_ratio"] = reduction * direction
        elif driver == "roboclaw":
            address, channel = int(axis["address"]), axis["channel"]
            if address not in ROBOCLAW_ADDRESSES or channel not in ("M1", "M2"):
                raise RuntimeError(f"{where}: address must be 128-135 and channel M1/M2")
            if (address, channel) in seen_channels:
                raise RuntimeError(f"{where}: {address}/{channel} already used")
            seen_channels.add((address, channel))
            qppr = float(axis["encoder_cpr"]) * reduction
            if abs(qppr - round(qppr)) > 1e-6:
                # The RoboClaw driver only accepts an integer counts-per-joint-rev.
                axis["qppr_rounding"] = qppr
            axis["qppr"] = int(round(qppr)) * direction
        else:
            raise RuntimeError(f"{where}: driver must be 'pxx' or 'roboclaw', got {driver!r}")
    return axes


def _joint_xml(name, params, command, states):
    lines = [f'    <joint name="{name}">']
    lines += [f'      <param name="{k}">{v}</param>' for k, v in params.items()]
    lines.append(f'      <command_interface name="{command}"/>')
    lines += [f'      <state_interface name="{s}"/>' for s in states]
    lines.append("    </joint>")
    return "\n".join(lines)


def _robot_description(can_axes, can_interface, rc_axes, roboclaw_port, roboclaw_baud):
    systems = []
    if can_axes:
        joints = "\n".join(
            _joint_xml(a["joint"], {"node_id": a["node_id"], "gear_ratio": a["gear_ratio"]},
                       "position", ["position", "velocity"])
            for a in can_axes)
        systems.append(f"""  <ros2_control name="ArmCan" type="system">
    <hardware>
      <plugin>pxx/PxxSystemHardware</plugin>
      <param name="can_interface">{can_interface}</param>
    </hardware>
{joints}
  </ros2_control>""")
    if rc_axes:
        if roboclaw_port == "mock":
            hardware = """      <plugin>mock_components/GenericSystem</plugin>
      <param name="calculate_dynamics">true</param>"""
        else:
            hardware = f"""      <plugin>roboclaw_hardware_interface/RoboClawHardwareInterface</plugin>
      <param name="serial_port">{roboclaw_port}</param>
      <param name="baud_rate">{roboclaw_baud}</param>"""
        joints = "\n".join(
            _joint_xml(a["joint"],
                       {"address": a["address"], "qppr": a["qppr"], "motor_type": a["channel"]},
                       "velocity", ["position"])
            for a in rc_axes)
        systems.append(f"""  <ros2_control name="ArmRoboClaw" type="system">
    <hardware>
{hardware}
    </hardware>
{joints}
  </ros2_control>""")
    body = "\n".join(systems)
    return f"""<?xml version="1.0"?>
<robot name="billee_arm">
  <link name="arm_base_link"/>
{body}
</robot>
"""


def _controllers_file(template_path, can_axes, rc_axes):
    with open(template_path) as f:
        params = yaml.safe_load(f)
    cm = params["controller_manager"]["ros__parameters"]
    for controller, axes in (("arm_can_controller", can_axes),
                             ("arm_roboclaw_controller", rc_axes)):
        if axes:
            params[controller]["ros__parameters"]["joints"] = [a["joint"] for a in axes]
        else:
            cm.pop(controller, None)
            params.pop(controller, None)
    out = tempfile.NamedTemporaryFile(
        "w", prefix="arm_controllers_", suffix=".yaml", delete=False)
    yaml.safe_dump(params, out)
    out.close()
    return out.name


def launch_setup(context):
    share = get_package_share_directory("arm_bringup")
    axes_file = LaunchConfiguration("axes_file").perform(context) or \
        os.path.join(share, "config", "arm_axes.yaml")
    controllers_template = LaunchConfiguration("controllers_file").perform(context) or \
        os.path.join(share, "config", "arm_controllers.yaml")
    can_interface = LaunchConfiguration("can_interface").perform(context).strip()
    roboclaw_port = LaunchConfiguration("roboclaw_port").perform(context).strip()
    roboclaw_baud = int(LaunchConfiguration("roboclaw_baud").perform(context))

    axes = _load_axes(axes_file)
    can_axes = [a for a in axes if a["driver"] == "pxx"] \
        if can_interface.lower() not in DISABLED else []
    rc_axes = [a for a in axes if a["driver"] == "roboclaw"] \
        if roboclaw_port.lower() not in DISABLED else []
    if not can_axes and not rc_axes:
        raise RuntimeError("arm.launch.py: both halves of the arm are disabled; nothing to run")

    actions = [LogInfo(msg=(
        f"[arm] axes file: {axes_file}\n"
        f"[arm] CAN axes ({can_interface}): {[a['joint'] for a in can_axes] or 'disabled'}\n"
        f"[arm] RoboClaw axes ({roboclaw_port}): {[a['joint'] for a in rc_axes] or 'disabled'}"))]
    for axis in can_axes + rc_axes:
        if not axis.get("verified", False):
            actions.append(LogInfo(msg=(
                f"[arm] WARNING: axis '{axis['joint']}' is not verified; its constants in "
                "arm_axes.yaml are placeholders (see docs/ARM_BRINGUP_GUIDE.md section 6).")))
        if "qppr_rounding" in axis:
            actions.append(LogInfo(msg=(
                f"[arm] WARNING: axis '{axis['joint']}' counts per joint rev "
                f"{axis['qppr_rounding']:.3f} rounded to {abs(axis['qppr'])} (RoboClaw driver "
                "takes an integer); expect a small scale error.")))

    if can_axes and can_interface.startswith("vcan"):
        actions.append(ExecuteProcess(
            cmd=[os.path.join(get_package_prefix("pxx"), "lib", "pxx", "pxx_vcan_emulator.py"),
                 "--interface", can_interface,
                 "--nodes", *[str(a["node_id"]) for a in can_axes]],
            output="screen"))

    description = _robot_description(
        can_axes, can_interface, rc_axes, roboclaw_port, roboclaw_baud)
    controllers = _controllers_file(controllers_template, can_axes, rc_axes)

    spawn = ["joint_state_broadcaster"]
    if can_axes:
        spawn.append("arm_can_controller")
    if rc_axes:
        spawn.append("arm_roboclaw_controller")

    actions += [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{"robot_description": description}, controllers],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[*spawn, "--controller-manager", "/controller_manager"],
            output="screen",
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "can_interface", default_value="can1",
            description="SocketCAN interface for the P17/P20 axes: canN = real bus, "
            "vcanN = virtual bus + CANopen emulator, mock = pxx loopback backend, "
            "none = leave the CAN axes out."),
        DeclareLaunchArgument(
            "roboclaw_port", default_value="/dev/roboclaw",
            description="Serial device of the shared RoboClaw UART (udev symlink, see "
            "docs/ARM_BRINGUP_GUIDE.md section 1.1). mock = mock_components/GenericSystem, "
            "none = leave the RoboClaw axes out."),
        DeclareLaunchArgument(
            "roboclaw_baud", default_value="115200",
            description="Packet serial baud rate; must match every RoboClaw's Motion Studio "
            "setting (factory default 38400). See the guide section 1.5 for the timing budget."),
        DeclareLaunchArgument(
            "axes_file", default_value="",
            description="Physical-constants YAML (default: arm_bringup/config/arm_axes.yaml)."),
        DeclareLaunchArgument(
            "controllers_file", default_value="",
            description="Controller template YAML "
            "(default: arm_bringup/config/arm_controllers.yaml)."),
        OpaqueFunction(function=launch_setup),
    ])
