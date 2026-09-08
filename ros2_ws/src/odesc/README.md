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

```
                       ┌────────────────────────────┐
 /diff_drive_controller │  diff_drive_controller     │  /odom, odom->base_link TF
 /cmd_vel_unstamped ───►│  (same config either way)  ├───►
                       └──────────┬─────────────────┘
                                  │ wheel-joint rad, rad/s
              ┌───────────────────┴───────────────────┐
     use_sim:=true                              use_sim:=false
   ign_ros2_control (Gazebo)              OdescSystemHardware (this pkg)
                                            │ SocketCAN, motor turns / turns·s⁻¹
                                            ▼
                                   6× ODESC V4.2 + NEO on can0
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
here — `read()`/`write()` apply no per-side sign flip. If a bench test shows the
ODESC reports the physically-mirrored sign on one side, that correction is added
deliberately then, not guessed now.

## CAN command subset used

CANSimple, ODrive firmware v0.5.x. Arbitration ID = `(node_id << 5) | cmd_id`
(11-bit standard frame). Command IDs come from
[`include/odesc/constants.hpp`](include/odesc/constants.hpp) — do not redefine
them elsewhere.

| cmd | name | direction | payload |
|---|---|---|---|
| `0x07` | `Set_Axis_Requested_State` | TX | `uint32` LE axis state: `8` (CLOSED_LOOP_CONTROL) on activate, `1` (IDLE) on deactivate/cleanup/shutdown |
| `0x09` | `Get_Encoder_Estimates` | RX (cyclic) | two LE `float32`: pos [0:4], vel [4:8], in **motor-shaft** turns / turns·s⁻¹ |
| `0x0D` | `Set_Input_Vel` | TX (per `write()`) | LE `float32` Input_Vel [0:4]; Input_Torque_FF [4:8] left `0` |

Axis-state values (`8`, `1`) are **not** in `constants.hpp` (which only carries
command IDs) — they are defined locally in `src/odesc.cpp`.

`Get_Encoder_Estimates` is consumed as a **cyclic** broadcast: configure each
ODESC with `axis0.config.can.encoder_rate_ms > 0` (e.g. `10`) during firmware
bring-up. The driver does not poll with RTR frames.

## Parameters (URDF `<ros2_control>` block)

Set in
[`robot_description/urdf/ros2_control.urdf.xacro`](../robot_description/urdf/ros2_control.urdf.xacro),
inside `<xacro:unless value="${use_sim}">`:

| scope | param | default | notes |
|---|---|---|---|
| `<hardware>` | `can_interface` | `can0` | SocketCAN interface. Confirm against the actual Jetson / USB-CAN enumeration. Special values `mock` / `none` → **mock mode** (below). Overridable from the launch file: `real.launch.py can_interface:=…`. |
| `<hardware>` | `gear_ratio` | `48.0` | Motor-shaft turns per wheel turn. `48.0` for the ODESC V4.2 + NEO REV v1.1 drivetrain (team, 2026-09-06). One-line override (`real.launch.py gear_ratio:=…`), not a code change. Still validated by the §5.4 drive-a-known-distance bench test. |
| per `<joint>` | `node_id` | *(required)* | CAN node ID 0–5. Canonical map: [`config/node_map.yaml`](config/node_map.yaml). |

### Mock mode (no CAN, no motor hardware)

Set `can_interface` to `mock` (or `none`) — e.g.
`ros2 launch chassis_bringup real.launch.py can_interface:=mock` — and the plugin
opens **no** CAN socket. `read()` feeds the commanded wheel velocity back through
the same gear-ratio round-trip the real path uses and integrates position, so the
full **encoder-feedback → `diff_drive_controller` → `/odom` → TF → RViz/Foxglove**
pipeline runs with no ODESC/NEO present. Use it to bring up and view the real
(non-Gazebo) control stack on a bench Jetson, or to regression-test the launch
graph. It is a perfect-tracking loopback, not a physics model — for dynamics use
the Gazebo backend (`sim_gz.launch.py`).

### vCAN ODrive emulator (CAN-path test, no motor hardware)

[`scripts/odesc_vcan_emulator.py`](scripts/odesc_vcan_emulator.py) is a small,
Linux-only ODrive CANSimple peer for the driver. Unlike `mock` mode, the driver
opens a real SocketCAN socket and exchanges frames; this verifies the CAN
arbitration IDs, `Set_Input_Vel` payload encoding, and encoder-feedback parsing.

It emulates nodes 0–5 and, at 100 Hz, publishes cyclic
`Get_Encoder_Estimates` (`0x09`) frames. Each node's motor-shaft position is
integrated from the most recently received `Set_Input_Vel` (`0x0D`) command.
The ODESC driver continues to perform its normal 48:1 motor-turns ↔ wheel-radian
conversion, so the emulator communicates only in motor-shaft turns and turns/s.

Create the virtual CAN interface in the same Linux network namespace as the ROS
processes, then either start the emulator directly or let the Gazebo shadow
launch do it:

```bash
# From the repository root. Requires sudo; safe to run again.
tooling/can-up vcan0

# Direct protocol / driver test (terminal 1):
ros2 run odesc odesc_vcan_emulator.py --interface vcan0

# Start the real ODESC controller path (terminal 2):
ros2 launch chassis_bringup real.launch.py can_interface:=vcan0
```

For the Nav2 integration test, `sim_gz.launch.py odesc_shadow:=true` starts the
emulator itself and mirrors Gazebo's controller command through an isolated
ODESC controller manager. Monitor the traffic with `candump -L vcan0` (from the
`can-utils` package), and remove the interface when finished:

```bash
tooling/can-up down vcan0
```

This is a deterministic protocol emulator, not an ODrive firmware or vehicle
simulation. It does not model arming/axis-state transitions, faults, limits,
latency, torque/current control, encoder noise, wheel slip, or CAN bus errors.
Use it to prove command/feedback integration; use Gazebo for vehicle dynamics
and hardware tests for ODrive behaviour.

### Canonical node map

[`config/node_map.yaml`](config/node_map.yaml) is the single source of truth for
node ID ↔ joint ↔ wheel position and for the `gear_ratio` value. Plain
xacro cannot parse it, so the URDF `node_id` params are hand-transcribed to
match — keep the two in sync.

| node_id | joint | wheel |
|---|---|---|
| 0 | `joint_wheel_l3` | Front Left |
| 1 | `joint_wheel_l2` | Mid Left |
| 2 | `joint_wheel_l1` | Rear Left |
| 3 | `joint_wheel_r1` | Front Right |
| 4 | `joint_wheel_r2` | Mid Right |
| 5 | `joint_wheel_r3` | Rear Right |

> The left side is numbered rear→front and the right side front→rear in the URDF,
> so `l1` and `r1` are not the same physical position. Match by wheel position.

## Lifecycle

- `on_init` – parse params, validate each joint has exactly one `velocity`
  command interface and both `position`+`velocity` state interfaces.
- `on_activate` – open the SocketCAN socket, start the RX thread, send
  `Set_Axis_Requested_State → CLOSED_LOOP_CONTROL` to all six nodes.
  *(mock mode: zero the state, no socket, no traffic.)*
- `read` – latest cyclic `Get_Encoder_Estimates` per node → wheel-joint state.
  *(mock mode: gear-ratio loopback of the command + position integration.)*
- `write` – commanded wheel velocity → `Set_Input_Vel` per node.
  *(mock mode: no-op; the command is consumed in `read`.)*
- `on_deactivate` / `on_cleanup` / `on_shutdown` – `Set_Axis_Requested_State →
  IDLE` on all nodes, stop the RX thread, close the socket.

## Platform note

The SocketCAN code is Linux-only. On non-Linux (e.g. a Mac dev build of the
workspace) the plugin still compiles and registers, but `on_activate()` fails
cleanly with a clear message — use the `use_sim:=true` Gazebo backend there.

## Selecting this backend

```bash
ros2 launch chassis_bringup real.launch.py      # real hardware, no Gazebo
ros2 launch chassis_bringup sim_gz.launch.py    # Gazebo, unchanged
```
