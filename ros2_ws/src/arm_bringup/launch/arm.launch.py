"""Bring up everything on the BILLEE arm: P17/P20 (CAN) and RoboClaw (UART) axes.

Everything is generated from config/arm_axes.yaml (the physical constants), so
adding, renaming or re-gearing an axis is a YAML edit only.

    # real hardware (arm CAN bus on can3, RoboClaws on the /dev/roboclaw UART):
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
generated URDF only carries the <ros2_control> blocks. /arm/joint_states is published.

Namespace: /arm (/arm/controller_manager, /arm/arm_can_controller/joint_trajectory and
/arm/arm_can_controller/follow_joint_trajectory, /arm/arm_roboclaw_controller/commands,
/arm/joint_states), apart from the drivetrain's /controller_manager and /joint_states.
"""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction,
                            Shutdown)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DISABLED = ("none", "off", "false", "")
# Everything of the arm's ros2_control lives under this namespace: /arm/controller_manager,
# /arm/arm_can_controller/..., /arm/joint_states. The drivetrain (rover.launch.py) has its own
# /controller_manager and /joint_states; sharing the name let its spawner reach the arm's.
NAMESPACE = "arm"
ROBOCLAW_ADDRESSES = range(128, 136)
# BasicMicro USB RoboClaws enumerate with the Atmel VID/PID and no USB serial number, so the
# units are told apart by their packet-serial address (set in Motion Studio), not by port.
ROBOCLAW_USB_IDS = {("03eb", "2404")}


def _crc16(data):
    """CRC16-CCITT/XMODEM, as the RoboClaw packet serial protocol uses."""
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def _roboclaw_usb_ports():
    """/dev/ttyACMn of every USB RoboClaw plugged in (sorted)."""
    ports = []
    base = "/sys/class/tty"
    for name in sorted(os.listdir(base)) if os.path.isdir(base) else []:
        if not name.startswith("ttyACM"):
            continue
        dev = os.path.realpath(os.path.join(base, name, "device"))
        ids = []
        for path in (dev, os.path.dirname(dev)):  # interface, then the USB device
            try:
                with open(os.path.join(path, "idVendor")) as v, \
                        open(os.path.join(path, "idProduct")) as pr:
                    ids = [(v.read().strip(), pr.read().strip())]
                    break
            except OSError:
                continue
        if ids and ids[0] in ROBOCLAW_USB_IDS:
            ports.append(f"/dev/{name}")
    return ports


def _roboclaw_answers(port, address, timeout=0.1):
    """True if `address` answers on `port`: read main battery voltage (command 24), CRC-checked.
    Read-only."""
    import select
    import termios

    try:
        fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError:
        return False
    try:
        attrs = termios.tcgetattr(fd)
        attrs[0] = attrs[1] = attrs[3] = 0  # raw: no input/output/local processing
        attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        attrs[4] = attrs[5] = termios.B115200  # ignored by USB CDC, needed for a valid setting
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        os.write(fd, bytes([address, 24]))
        reply = b""
        deadline = timeout
        while len(reply) < 4:
            ready, _, _ = select.select([fd], [], [], deadline)
            if not ready:
                return False
            reply += os.read(fd, 4 - len(reply))
        crc = _crc16(bytes([address, 24]) + reply[:2])
        return reply[2:4] == bytes([crc >> 8, crc & 0xFF])
    except OSError:
        return False
    finally:
        os.close(fd)


def _find_roboclaws(addresses, wait_s=3.0):
    """-> ({address: port}, {address: [ports]} for addresses answering on more than one port).

    Every configured address is asked on every USB RoboClaw port. A unit plugged in or powered
    up just before the launch can take a moment to answer (and to enumerate), so missing
    addresses are retried for up to wait_s. An address answering on two ports (two units left
    on the same address, e.g. both on the factory 128) is reported, not guessed at."""
    import time

    deadline = time.monotonic() + wait_s
    while True:
        seen = {}
        for port in _roboclaw_usb_ports():
            for address in sorted(addresses):
                if _roboclaw_answers(port, address):
                    seen.setdefault(address, []).append(port)
        found = {a: ports[0] for a, ports in seen.items() if len(ports) == 1}
        duplicates = {a: ports for a, ports in seen.items() if len(ports) > 1}
        if duplicates or set(found) >= set(addresses) or time.monotonic() >= deadline:
            return found, duplicates
        time.sleep(0.5)


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

        # output: duty = a RoboClaw channel driven by duty with no encoder (e.g. a solenoid).
        axis["output"] = str(axis.get("output", "motor"))
        if axis["output"] not in ("motor", "duty") or (
                axis["output"] == "duty" and driver != "roboclaw"):
            raise RuntimeError(f"{where}: output must be motor, or duty (roboclaw only)")
        if axis["output"] == "duty":
            address, channel = int(axis["address"]), axis["channel"]
            if address not in ROBOCLAW_ADDRESSES or channel not in ("M1", "M2"):
                raise RuntimeError(f"{where}: address must be 128-135 and channel M1/M2")
            if (address, channel) in seen_channels:
                raise RuntimeError(f"{where}: {address}/{channel} already used")
            seen_channels.add((address, channel))
            continue

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

        if "limit_switch" in axis:
            try:
                limit_switch = int(axis["limit_switch"])
            except (TypeError, ValueError) as e:
                raise RuntimeError(f"{where}: limit_switch must be an integer GPIO ID") from e
            if limit_switch < 0:
                raise RuntimeError(f"{where}: limit_switch must be non-negative")
            axis["limit_switch"] = limit_switch
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
            _joint_xml(a["joint"], {
                       "node_id": a["node_id"], "gear_ratio": a["gear_ratio"],
                       **({"limit_switch": a["limit_switch"]} if "limit_switch" in a else {})},
                       "position", ["position", "velocity"])
            for a in can_axes)
        systems.append(f"""  <ros2_control name="ArmCan" type="system">
    <hardware>
      <plugin>pxx/PxxSystemHardware</plugin>
      <param name="can_interface">{can_interface}</param>
    </hardware>
{joints}
  </ros2_control>""")
    # One hardware system per serial port: a USB RoboClaw is its own port (roboclaw_port:=auto
    # gives each axis the port its address answered on); a shared UART carries them all.
    ports = []
    for a in rc_axes:
        if a["port"] not in ports:
            ports.append(a["port"])
    for port in ports:
        on_port = [a for a in rc_axes if a["port"] == port]
        if port == "mock":
            name = "ArmRoboClaw"
            hardware = """      <plugin>mock_components/GenericSystem</plugin>
      <param name="calculate_dynamics">true</param>"""
        else:
            addresses = sorted({a["address"] for a in on_port})
            name = "ArmRoboClaw" + "_".join(str(x) for x in addresses)
            hardware = f"""      <plugin>roboclaw_hardware_interface/RoboClawHardwareInterface</plugin>
      <param name="serial_port">{port}</param>
      <param name="baud_rate">{roboclaw_baud}</param>"""
        joints = "\n".join(
            _joint_xml(a["joint"], {"address": a["address"], "motor_type": a["channel"]},
                       "duty", ["duty"])
            if a["output"] == "duty" else
            _joint_xml(a["joint"], {
                       "address": a["address"], "qppr": a["qppr"], "motor_type": a["channel"],
                       "max_velocity": float(a["limits"]["max_velocity"]),
                       **({"limit_switch": a["limit_switch"]} if "limit_switch" in a else {})},
                       "velocity", ["position"])
            for a in on_port)
        systems.append(f"""  <ros2_control name="{name}" type="system">
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
    motors = [a for a in rc_axes if a["output"] == "motor"]
    duty = [a for a in rc_axes if a["output"] == "duty"]
    for controller, axes in (("arm_can_controller", can_axes),
                             ("arm_roboclaw_controller", motors),
                             ("arm_solenoid_controller", duty)):
        if axes:
            params[controller]["ros__parameters"]["joints"] = [a["joint"] for a in axes]
        else:
            cm.pop(controller, None)
            params.pop(controller, None)
    # Parameter-file keys are node names; namespaced nodes need the full name.
    params = {f"/{NAMESPACE}/{name}": value for name, value in params.items()}
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
    skipped = []
    if roboclaw_port.startswith("/dev/") and not os.path.exists(roboclaw_port):
        # No RoboClaw plugged in (e.g. the CAN joints on the bench): run the CAN half alone
        # rather than failing the whole ros2_control_node on a missing serial port.
        skipped.append(LogInfo(msg=f"[arm] WARNING: {roboclaw_port} doesn't exist; RoboClaw axes "
                                   "disabled (roboclaw_port:=none silences this)"))
        roboclaw_port = "none"
    can_axes = [a for a in axes if a["driver"] == "pxx"] \
        if can_interface.lower() not in DISABLED else []
    rc_axes = [a for a in axes if a["driver"] == "roboclaw"] \
        if roboclaw_port.lower() not in DISABLED else []
    if rc_axes and roboclaw_port == "auto":
        # USB RoboClaws: find which port each configured address answers on.
        found, duplicates = _find_roboclaws({a["address"] for a in rc_axes})
        for address, ports in sorted(duplicates.items()):
            skipped.append(LogInfo(msg=(
                f"[arm] WARNING: RoboClaw address {address} answers on {len(ports)} ports "
                f"({', '.join(ports)}): two units share it, so its axes are left out. Give each "
                "unit its own address in Motion Studio (write the settings to the unit, then "
                "power-cycle it).")))
        missing = sorted({a["address"] for a in rc_axes} - set(found) - set(duplicates))
        if missing:
            skipped.append(LogInfo(msg=(
                f"[arm] WARNING: RoboClaw address(es) {missing} didn't answer on any USB "
                f"RoboClaw port ({_roboclaw_usb_ports() or 'none plugged in'}); their axes are "
                "left out. Check USB, the battery and the address in Motion Studio "
                "(roboclaw_probe.py --port /dev/ttyACMn lists who answers).")))
        rc_axes = [dict(a, port=found[a["address"]]) for a in rc_axes if a["address"] in found]
        for address, port in sorted(found.items()):
            skipped.append(LogInfo(msg=f"[arm] RoboClaw {address} on {port}"))
    else:
        rc_axes = [dict(a, port=roboclaw_port) for a in rc_axes]
    if not can_axes and not rc_axes:
        raise RuntimeError("arm.launch.py: both halves of the arm are disabled; nothing to run")

    actions = skipped + [LogInfo(msg=(
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
    if any(a["output"] == "motor" for a in rc_axes):
        spawn.append("arm_roboclaw_controller")
    if any(a["output"] == "duty" for a in rc_axes):
        spawn.append("arm_solenoid_controller")

    actions += [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=NAMESPACE,
            parameters=[{"robot_description": description}, controllers],
            output="screen",
            # Without it a dead driver (e.g. a drive that won't enable) left the spawner waiting
            # forever and the launch looking alive; this ends the whole launch so it shows.
            on_exit=Shutdown(reason="ros2_control_node exited: the arm driver is not running "
                                    "(see its FATAL line above)"),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            namespace=NAMESPACE,
            arguments=[*spawn, "--controller-manager", f"/{NAMESPACE}/controller_manager"],
            output="screen",
        ),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "can_interface", default_value="can3",
            description="SocketCAN interface for the P17/P20 axes: canN = real bus, "
            "vcanN = virtual bus + CANopen emulator, mock = pxx loopback backend, "
            "none = leave the CAN axes out."),
        DeclareLaunchArgument(
            "roboclaw_port", default_value="auto",
            description="auto = USB RoboClaws: each unit is found by its packet-serial address "
            "on the USB RoboClaw ports (one hardware system per unit). A /dev/... path = one "
            "shared packet-serial UART for every unit (docs/ARM_BRINGUP_GUIDE.md section 1.1). "
            "mock = mock_components/GenericSystem, none = leave the RoboClaw axes out."),
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
