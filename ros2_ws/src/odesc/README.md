# odesc

`ros2_control` hardware interface for the BILLEE drivetrain's six ODrive/ODESC
motor controllers, spoken to over a **raw Linux SocketCAN** socket (no extra ROS
CAN dependency).

## What it is

`OdescSystemHardware` is a `hardware_interface::SystemInterface` plugin. It is the
real-hardware counterpart to the Gazebo `ign_ros2_control/IgnitionSystem` backend:
`diff_drive_controller` is unchanged and never knows which one is loaded — it only
ever sees the six wheel-joint `position`/`velocity` state interfaces and one
`velocity` command interface per joint.

`diff_drive_controller` publishes wheel odometry on `/diff_drive_controller/odom`
but not the `odom → base_footprint` TF. That TF belongs to the EKF (see
[docs/architecture.md](../../../docs/architecture.md#odometry-and-tf)).

```
                       ┌────────────────────────────┐
 /diff_drive_controller │  diff_drive_controller     │  /diff_drive_controller/odom
 /cmd_vel_unstamped ───►│  (same config either way)  ├───► (no TF: enable_odom_tf: false)
                       └──────────┬─────────────────┘
                                  │ wheel-joint rad, rad/s
              ┌───────────────────┴───────────────────┐
     use_sim:=true                              use_sim:=false
   ign_ros2_control (Gazebo)              OdescSystemHardware (this pkg)
                                            │ SocketCAN, motor turns / turns·s⁻¹
                                            ▼
                                   6× ODESC V4.2 + NEO on can3
```

## Gear-ratio conversion (the only place it lives)

`diff_drive_controller` and `controllers.yaml` deal purely in wheel-joint units;
`wheel_radius` / `wheel_separation` do **not** change. The motor↔wheel conversion
is entirely internal to this plugin:

```
read()   wheel_rad_s      = motor_turns_per_sec * 2π / gear_ratio
write()  motor_turns_per_sec = wheel_rad_s       * gear_ratio / 2π
```

`gear_ratio` is **48.0** for the ODESC V4.2 + NEO REV v1.1 drivetrain (set by the
team 2026-09-06). It is one overridable parameter — see the parameter table below.

Left/right mirroring is handled by `diff_drive_controller`'s wheel lists, **not**
here. If a bench test shows that one ODESC's encoder/command sign is physically
reversed, add an `invert` joint parameter for that motor. The driver applies the
same sign correction to command and feedback, preserving ROS wheel-joint
semantics. Do not add it speculatively.

## CAN command subset used

CANSimple. Arbitration ID = `(node_id << 5) | cmd_id` (11-bit standard frame).
Message layouts come from
[`include/odesc/can_simple_messages.hpp`](include/odesc/can_simple_messages.hpp).

The ODESC boards run **pre-0.5.2 ODrive firmware**: their configs
([`configs/odesc/*.json`](../../../configs/odesc)) have `axis0.config.can_node_id`
and `can_heartbeat_rate_ms`, not the later `axis0.config.can.*` block. On this
firmware the heartbeat is the only cyclic message, and there is no
`encoder_rate_ms` setting: encoder estimates are sent **only in reply to an RTR
request**. So the driver polls every axis each control cycle.

| cmd | name | direction | when | payload |
|---|---|---|---|---|
| `0x01` | `Heartbeat` | RX | ODESC sends at 10 Hz | `uint32` axis error [0:4], axis state [4] (used by the health check) |
| `0x07` | `Set_Axis_Requested_State` | TX | activate: `8` (closed loop); deactivate / error / cleanup: `1` (IDLE) | `uint32` LE, 4-byte frame |
| `0x09` | `Get_Encoder_Estimates` | TX RTR, then RX | RTR request every `write()`; the reply is read in the next `read()` | two LE `float32`: pos [0:4], vel [4:8], **motor-shaft** turns / turns·s⁻¹ |
| `0x0B` | `Set_Controller_Mode` | TX | activate | velocity control, `VEL_RAMP` input mode |
| `0x0D` | `Set_Input_Vel` | TX | **every** `write()` | LE `float32` Input_Vel [0:4]; Input_Torque_FF [4:8] left `0` |
| `0x18` | `Clear_Errors` | TX | activate | 1 byte, `0` |

Per cycle that is 12 frames out (6 velocity commands, then 6 encoder requests) and
6 replies in: about 13–15% of the 500 kbit/s bus at 30 Hz including heartbeats.
The kernel's default CAN transmit queue (10 frames) is too small for that burst;
[`tooling/can-up`](../../../tooling/can-up) sets `txqueuelen 100`.

Error codes are logged as hex only: the names in `odrive_enums.h` are for 0.6
firmware and don't match this one.

## Parameters (URDF `<ros2_control>` block)

Set in
[`robot_description/urdf/ros2_control.urdf.xacro`](../robot_description/urdf/ros2_control.urdf.xacro),
inside `<xacro:unless value="${use_sim}">`:

| scope | param | default | notes |
|---|---|---|---|
| `<hardware>` | `can_interface` | `can3` | SocketCAN interface. Overridable from the launch file: `real.launch.py can_interface:=…`. (`mock` / `none` is currently broken, see below.) |
| `<hardware>` | `fault_policy` | `warn` | What happens when a wheel is unhealthy (see [Wheel health](#wheel-health-and-fault_policy)): `warn` logs it and keeps driving; `fault` stops the drivetrain. Overridable: `rover.launch.py` / `real.launch.py fault_policy:=fault`. |
| `<hardware>` | `gear_ratio` | `48.0` | Motor-shaft turns per wheel turn. `48.0` for the ODESC V4.2 + NEO REV v1.1 drivetrain (team, 2026-09-06). One-line override (`real.launch.py gear_ratio:=…`), not a code change. Not yet confirmed on hardware: drive a measured straight distance and compare it with `/diff_drive_controller/odom`. |
| per `<joint>` | `node_id` | *(required)* | CAN node ID 0–63 (the rover uses 2–7). Canonical map: [`config/node_map.yaml`](config/node_map.yaml). |
| per `<joint>` | `invert` | `false` | Optional. Set `true` only after a wheels-up direction test proves that this axis's physical sign is opposite the URDF wheel-joint convention. |

For example, to invert one confirmed-reversed motor, add this alongside that
joint's `node_id` parameter in `ros2_control.urdf.xacro`:

```xml
<param name="invert">true</param>
```

### Command refresh and ODrive watchdog

`Set_Input_Vel` is sent to every axis on every `write()`, so a lost frame is
replaced on the next cycle. A single failed send (for example `ENOBUFS`) is
logged and retried next cycle; it does not stop the drivetrain. The checked-in
ODESC profiles still have the CAN watchdog disabled. Because commands now
arrive every cycle, it can be enabled (e.g. `watchdog_timeout` 0.5 s) without
tripping during steady driving.

### Wheel health and `fault_policy`

While active, `read()` checks every wheel. A wheel is **unhealthy** if:

- no encoder reply for 0.25 s;
- no heartbeat for 0.5 s;
- the heartbeat reports a non-zero axis error, or a state other than 8 (closed loop)
  (both checked from 1 s after activation, so `Clear_Errors` and the
  closed-loop request can take effect);
- velocity commands to it have been failing for 0.25 s.

Each unhealthy wheel logs an error at most once per second, naming the joint, the
node ID and the reasons.

- `fault_policy` `warn` (default): `read()` still returns OK and the rover keeps
  driving on the other wheels.
- `fault_policy` `fault`: `read()` returns ERROR. ros2_control calls
  `on_error`, which sends zero velocity then IDLE to every axis and closes the
  socket, and the controllers deactivate. Restart the launch to recover.

### Mock mode (no CAN, no motor hardware)

> **Currently broken.** The new driver (`src/odesc.cpp`) has no mock path, so
> `can_interface:=mock` fails in `on_configure`. Restoring it (by reusing
> `MockDriveBackend`) is deferred. Until then use the vCAN emulator below.

Set `can_interface` to `mock` (or `none`) — e.g.
`ros2 launch chassis_bringup real.launch.py can_interface:=mock` — and the plugin
opens **no** CAN socket. `read()` feeds the commanded wheel velocity back through
the same gear-ratio round-trip the real path uses and integrates position, so the
full **encoder-feedback → `diff_drive_controller` → `/diff_drive_controller/odom` + `/joint_states` → RViz/Foxglove**
pipeline runs with no ODESC/NEO present. Use it to bring up and view the real
(non-Gazebo) control stack on a bench Jetson, or to regression-test the launch
graph. It is a perfect-tracking loopback, not a physics model — for dynamics use
the Gazebo backend (`sim_gz.launch.py`).

### vCAN ODrive emulator (CAN-path test, no motor hardware)

[`scripts/odesc_vcan_emulator.py`](scripts/odesc_vcan_emulator.py) is a small,
Linux-only ODrive CANSimple peer for the driver. Unlike `mock` mode, the driver
opens a real SocketCAN socket and exchanges frames; this verifies the CAN
arbitration IDs, `Set_Input_Vel` payload encoding, and encoder-feedback parsing.

It emulates nodes 2–7 (`--nodes` to change) and behaves like the rover's firmware:

- a 10 Hz heartbeat whose axis state follows `Set_Axis_Requested_State`: IDLE (1)
  until the driver requests 8 (closed loop);
- `Get_Encoder_Estimates` (`0x09`) only in reply to an RTR request (default `--mode rtr`).
  `--mode cyclic` broadcasts at 100 Hz instead, like newer firmware;
- motor-shaft position integrated from the last `Set_Input_Vel` (`0x0D`), in closed loop only.

The ODESC driver does its normal 48:1 motor-turns ↔ wheel-radian conversion, so
the emulator works only in motor-shaft turns and turns/s.

Create the virtual CAN interface in the same Linux network namespace as the ROS
processes, then either start the emulator directly or let the Gazebo shadow
launch do it:

```bash
# On the host (not in the container), from the repository root. Requires sudo; safe to run again.
tooling/can-up vcan0

# Direct protocol / driver test (terminal 1):
ros2 run odesc odesc_vcan_emulator.py --interface vcan0

# Start the real ODESC controller path (terminal 2):
ros2 launch chassis_bringup real.launch.py can_interface:=vcan0
```

To check the CAN path while the simulated rover drives (by gamepad or by Nav2,
which `sim_gz.launch.py` starts with `navigation:=true`),
`sim_gz.launch.py odesc_shadow:=true` starts the emulator itself and mirrors
Gazebo's `cmd_vel` through an isolated, namespaced ODESC controller manager on
`vcan0`. Monitor the traffic with `candump -L vcan0` (from the
`can-utils` package), and remove the interface when finished:

```bash
tooling/can-up down vcan0
```

This is a deterministic protocol emulator, not an ODrive firmware or vehicle
simulation. Beyond the IDLE / closed-loop state it does not model faults, limits,
latency, torque/current control, encoder noise, wheel slip, or CAN bus errors.
To exercise the health check, stop the emulator: every wheel reports no encoder
reply and no heartbeat.
Use it to prove command/feedback integration; use Gazebo for vehicle dynamics
and hardware tests for ODrive behaviour.

### Canonical node map

[`config/node_map.yaml`](config/node_map.yaml) is the single source of truth for
node ID ↔ joint ↔ wheel position and for the `gear_ratio` value. Plain
xacro cannot parse it, so the URDF `node_id` params are hand-transcribed to
match — keep the two in sync.

| node_id | joint | wheel |
|---|---|---|
| 4 | `joint_wheel_l3` | Front Left |
| 7 | `joint_wheel_l2` | Mid Left |
| 2 | `joint_wheel_l1` | Rear Left |
| 3 | `joint_wheel_r1` | Front Right |
| 6 | `joint_wheel_r2` | Mid Right |
| 5 | `joint_wheel_r3` | Rear Right |

The node IDs are set on each board (`axis0.config.can_node_id` in
[`configs/odesc/motor_N.json`](../../../configs/odesc); `motor_1` is node 7, the
rest match their number). Each board's unused `axis1` also sends heartbeats,
mostly as node 1; the driver ignores them.

> The left side is numbered rear→front and the right side front→rear in the URDF,
> so `l1` and `r1` are not the same physical position. Match by wheel position.

## Lifecycle

The plugin `odesc/OdescSystemHardware` is implemented by
`odesc_ros2_control::OdescHardwareInterface` in [`src/odesc.cpp`](src/odesc.cpp):

- `on_configure`: open the SocketCAN socket.
- `on_activate`: per axis, `Clear_Errors`, velocity / `VEL_RAMP` mode, zero
  velocity, then closed loop.
- `read()`: drain received frames (encoder replies, heartbeats), then the health check.
- `write()`: `Set_Input_Vel` to every axis, then an encoder RTR to every axis.
- `on_deactivate` / `on_error`: zero velocity, then IDLE. `on_error` also closes the socket.
- `on_cleanup` / `on_shutdown`: close the socket.

`MockDriveBackend` / `SocketCanDriveBackend` (`src/*_drive_backend.cpp`) belong
to the previous driver design. They are still compiled but unused.

## Platform note

This package is Linux-only: it directly uses the kernel SocketCAN API and is
therefore built only when CMake targets Linux. Use the `use_sim:=true` Gazebo
backend on other platforms.

## Selecting this backend

```bash
ros2 launch chassis_bringup real.launch.py      # real hardware, no Gazebo
ros2 launch chassis_bringup sim_gz.launch.py    # Gazebo, unchanged
```
