# BILLEE Arm — Bring-up Guide

This guide covers every motor on the arm:

- the **P17 / P20** actuators, on a CANopen bus via the `pxx` driver;
- the **RoboClaw**-driven motors, on one shared TTL UART via `roboclaw_hardware_interface`.

All the code lives in `ros2_ws/src/arm_bringup/`. For the CANopen deep-dive (drive state
machine, node-ID assignment, raw SDO frames), see [PXX_BRINGUP_GUIDE.md](PXX_BRINGUP_GUIDE.md).

| deliverable | section | in the repo |
|---|---|---|
| 1. Connecting to the motors (RoboClaw setup, PID, soft limits) | [§1](#1-connecting-to-the-motors) | — |
| 2. Tuning | [§2](#2-tuning) | `tuning:` records in `arm_axes.yaml` |
| 3. Verifying bus data + message formats | [§3](#3-verifying-data-from-every-motor) | `pxx_probe.py`, `roboclaw_probe.py` |
| 4. Test speeds | [§4](#4-test-speeds-very-slow) | `test_velocity` in `arm_axes.yaml`, `arm_slow_jog.py` |
| 5. Launch | [§5](#5-launching-the-arm) | `launch/arm.launch.py` |
| 6. Physical constants | [§6](#6-physical-constants-arm_axesyaml) | `config/arm_axes.yaml` |

> **Every value in `arm_axes.yaml` is a placeholder until it is measured.** The launch file
> prints a warning for each axis still marked `verified: false`.

```
                Jetson
   ┌────────────────────────────────┐
   │  ros2_control_node             │
   │   ├─ ArmCan      (pxx)  ───────┼── can1 @ 1 Mbit/s ──┬── P20 (node 1) ── P17 (node 2) ── …
   │   │                            │   120 Ω each end    │
   │   └─ ArmRoboClaw (roboclaw) ───┼── UART (/dev/roboclaw) ── RoboClaw 128 ── RoboClaw 129 ── …
   └────────────────────────────────┘   packet serial, multi-unit mode, 115200 baud
```

Commands follow [RUN_GUIDE.md](RUN_GUIDE.md): run from `ros2_ws/`, with
`pixi run --environment l4t …` on the rover. Build once:

```bash
pixi run --environment l4t build      # builds pxx, roboclaw_*, arm_bringup
```

---

## 1. Connecting to the motors

### 1.1 RoboClaws: one shared TTL UART

Every RoboClaw sits on the same Jetson UART. Each unit has a unique **packet-serial
address** (128–135), so up to 8 units (16 motors) can share it. The driver talks to all of
them through one `serial_port`.

**Wiring:**

| Jetson (40-pin header UART) | RoboClaw (every unit) |
|---|---|
| TX | **S1** (RX in) — all units in parallel |
| RX | **S2** (TX out) — all units in parallel, needs **Multi-Unit Mode** (below) |
| GND | **GND** — common to all units and the Jetson |

- **Multi-Unit Mode** turns each RoboClaw's S2 into an open-drain output, so several units
  can share the Jetson's RX line. That line then needs **one pull-up resistor** to the
  Jetson's logic level. The Jetson header is **3.3 V and not 5 V tolerant**. Take the
  resistor value from the RoboClaw user manual's multi-unit section, and confirm the RoboClaw
  model's input threshold accepts 3.3 V.
- Use the header UART that is free on your carrier board, e.g. `/dev/ttyTHS1` on Orin
  Nano/NX dev kits. Make sure no serial console or `nvgetty` is running on it.
- Give the port a stable name and access rights. Create `/etc/udev/rules.d/99-roboclaw.rules`:
  ```
  KERNEL=="ttyTHS1", SYMLINK+="roboclaw", GROUP="dialout", MODE="0660"
  ```
  Then run `sudo udevadm control --reload && sudo udevadm trigger` and
  `sudo usermod -aG dialout $USER` (log out and in again). The launch file defaults to
  `/dev/roboclaw`.

### 1.2 RoboClaw configurator: BasicMicro Motion Studio

**Yes, there is a configurator tool: BasicMicro Motion Studio.** It is Windows-only and
connects **over USB**. Configure each RoboClaw on its own over USB from a Windows laptop,
save it to EEPROM, then wire it into the UART chain. Settings that matter, per unit:

| Motion Studio setting | set to | why |
|---|---|---|
| Control Mode | **Packet Serial** | The ROS driver speaks packet serial. |
| Packet Serial Address | **unique, 128–135**, matching `address` in `arm_axes.yaml` | Units on the shared UART are told apart by address. |
| Baud Rate | **115200**, the same on every unit (factory default is 38400; see [§1.5](#15-roboclaw-driver-notes)) | One UART, one baud rate. 38400 is too slow for more than one unit at 100 Hz. |
| Multi-Unit Mode | **on** | Open-drain S2 so several units share Jetson RX. |
| Encoder mode (M1/M2) | Quadrature (or Absolute, if fitted) | The driver needs encoder counts. |
| Serial Timeout | ~0.5 s, if your firmware exposes it (packet command 14) | **The motors stop if ROS stops talking.** Without it, a crashed ROS stack leaves the last speed running. |
| Max Current (M1/M2) | low for bring-up (≈ 30–50 % of the motor's rating) | A stall or bad command can't cook a motor or strip a gear. |
| Main battery min/max | match the arm supply | Brown-out / regen protection. |
| **Device → Write Settings** | after every change | Writes the settings to EEPROM. Otherwise they're lost at power-off. |

#### Can we do PID tuning in Motion Studio?

**Yes.** Motion Studio's velocity settings have an **auto-tune** that measures **QPPS**
(maximum counts/s) and sets the **velocity PID** for each channel. That loop is the one the
ROS driver uses: it sends *signed speed* commands (§3.2). Motion Studio also has position
PID and its own auto-tune, but the current ROS driver never uses position mode. See [§2.1](#21-roboclaw-motion-studio)
for the procedure.

#### Can we set soft limits in Motion Studio?

**Partly, and not in a way that protects us today.**

- Motion Studio's position settings have **Min / Max Position**. These clamp *position*
  commands only. The ROS driver sends *velocity* commands, so **these limits are not
  enforced** for the arm as it is driven now.
- What does work at the RoboClaw level: **limit switches.** Configure S4 / S5 as limit
  (or home) inputs per motor in Motion Studio, so a switch stops that direction in hardware.
  Use them on any axis that can hit a hard stop or wrap cables.
- The ROS-level limits in `arm_axes.yaml` (`limits.min_position / max_position /
  max_velocity`) are enforced by `arm_slow_jog.py` **only**. A raw topic or action command
  bypasses them.
- **RoboClaw positions are relative.** The driver sets each RoboClaw joint to **0 rad at
  launch**, wherever it is. Software limits only mean something if the arm starts from a
  known pose, or once a homing step is added.

### 1.3 Limits: what exists where

| layer | P17 / P20 | RoboClaw |
|---|---|---|
| hardware | Mechanical stops. Drive soft limit (`2003h` = `LIMT` to enable; the limit values are most likely CiA402 `607Dh:01/02`, so confirm in the P-series manual). | Limit switches on S4/S5. Max current. |
| drive firmware | Overspeed fault threshold `2024h`, following-error window `6065h`. | Serial timeout. Max current. |
| ROS | `arm_axes.yaml` limits via `arm_slow_jog.py`. | Same. **Relative to the pose at launch.** |

### 1.4 P17 / P20: CAN bus

Use a dedicated arm bus, separate from the drivetrain's `can0`. The drives default to 1 Mbit/s,
and the ODrive CAN IDs on `can0` collide with CANopen NMT/SYNC. The details are in
[PXX_BRINGUP_GUIDE.md §3](PXX_BRINGUP_GUIDE.md#3-bench-one-motor-raw-can-no-ros). In short:

```bash
~/billee-software-2027/tooling/can-up can1 1000000
```

- 120 Ω termination at **each** end of the bus (≈ 60 Ω between CAN-H and CAN-L, measured
  with the bus powered down).
- A common ground.
- Each drive gets a unique node ID (`2001h:01`, then save with `1010h:01`), matching
  `node_id` in `arm_axes.yaml`.

### 1.5 RoboClaw driver notes

`roboclaw_serial` and `roboclaw_hardware_interface` are **vendored**: they're normal
packages in `ros2_ws/src/`, not submodules. Upstream (dumbotics) was written for and tested
over USB. Our UART fixes are listed in each package's `VENDORED.md` and marked `BILLEE:` in
the code:

- **The port is set to `baud_rate`.** Upstream left it at `B0`, which only works over USB.
- **Reads wait for the whole reply,** and **input is flushed before each request,** so a
  late reply can't desync the next one. Upstream read one chunk, which over a UART is
  usually just the first byte.
- **Serial errors are logged and the cycle skipped.** Only 5 failures in a row (50 ms)
  report a hardware ERROR. Upstream crashed `ros2_control_node`, taking the CAN axes down too.
- **The motors stop on deactivate or Ctrl-C,** and are never driven while the hardware is
  inactive.

These were tested against a simulated RoboClaw that sends replies a byte at a time, **not
yet against real hardware**. Still as upstream:

- velocity command and position state only;
- position is **relative to launch**;
- there is no current or status reporting.

**Baud budget:** each RoboClaw costs about 24 bytes per control cycle (encoder read 2+10,
speed write 11+1), which is about 6 ms per unit at 38400 baud. Two units already exceed the
10 ms budget at 100 Hz. Set every unit to **115200** (about 2 ms per unit) in Motion Studio.
That's the launch default (`roboclaw_baud`); use the same rate for `roboclaw_probe.py --baud`.

---

## 2. Tuning

Tune bottom-up: drive loops first, then motion limits, then the ROS trajectory settings.
Always tune with the axis **supported or in the middle of its travel**, at the low bring-up
current limit.

### 2.1 RoboClaw (Motion Studio)

For each channel, over USB:

1. **Direction check.** Drive the motor slowly forward with duty (PWM) control in Motion
   Studio. The encoder count must **increase**. If it decreases, swap the motor leads or
   the encoder A/B (or the channel's encoder-reverse option). Fix this first: the PID can't
   work with reversed feedback.
2. **QPPS.** Run the velocity auto-tune, or run the motor at 100 % duty and read the max
   counts/s. Write the value into the velocity settings.
3. **Velocity PID.** Auto-tune, then check with a slow speed step in Motion Studio's graph:
   no oscillation, and it settles quickly.
4. **Record** QPPS, P/I/D and max current into the axis's `tuning:` block in
   `arm_axes.yaml`. The launch doesn't push them; the record is there so a replaced or reset
   unit can be restored.
5. **Device → Write Settings** (EEPROM).

### 2.2 P17 / P20

The drives' current, velocity and position loops run inside the actuator, and come
pre-tuned. Our protocol notes (`local_notes/canopen_protocol_descriptions.md`) don't list
the gain objects. If retuning is ever needed, use the manufacturer's tool or full manual.
Normally, tune what's around the loop instead:

- **Speed** comes from the trajectory timing (§4). In CSP mode the drive chases each target
  at full speed, so never send large position steps.
- **Following-error window `6065h`.** If slow moves trip `0x8611` (*position error too
  large*), the window is too tight for the load, or the gains are too soft. Raise it a
  little at a time. Don't disable it.
- **Overspeed threshold `2024h`.** Lower it during bring-up as a backstop (it raises a
  fault, not a clamp). Check its units in the manual.
- Only touch drive gains if following errors persist at test speed with a reasonable `6065h`.

### 2.3 ROS layer

`config/arm_controllers.yaml` → `arm_can_controller.constraints`: `goal_time` (slack before
a trajectory is aborted) and `stopped_velocity_tolerance`. You can add per-joint `goal` /
`trajectory` tolerances once the arm tracks well. Loose tolerances hide tuning problems,
so tighten them as each axis is verified.

---

## 3. Verifying data from every motor

Only the P17/P20s are on CAN. **The RoboClaws are on the UART, not CAN**, so each bus has
its own probe. Both probes are stdlib-only Python, installed with `arm_bringup`.

### 3.1 CAN (P17 / P20)

```bash
# with the arm launch STOPPED (scan sends SDOs):
pixi run --environment l4t ros2 run arm_bringup pxx_probe.py scan --interface can1
#   drives on the bus: [1, 2]
#   node 1:
#     status word 6041h      0x0250 (switch on disabled)
#     error code  603Fh      0x0000 (no error)
#     position    6064h      123456 counts = +1.884 motor rev
#     gear ratio  6091h:01   …

# while the arm launch is RUNNING (listen-only):
pixi run --environment l4t ros2 run arm_bringup pxx_probe.py rates   --interface can1
#   RPDO1 n1:100  SYNC:100  TPDO1 n1:100  TPDO1 n2:100  TPDO2 n1:100  TPDO2 n2:100
pixi run --environment l4t ros2 run arm_bringup pxx_probe.py monitor --interface can1
#   0.0102  080  SYNC
#   0.0104  181  TPDO1     node 1: position 123456 (+1.884 motor rev), velocity 0
#   0.0105  281  TPDO2     node 1: status 0x0427 (operation enabled)
```

**Pass:** every `node_id` in `arm_axes.yaml` shows up in `scan`. With the launch running,
`rates` shows **TPDO1 and TPDO2 at the update rate (100/s) for every node.** A node missing
from `rates` isn't sending feedback. The plain `candump -tz can1` (can-utils) works too.

**Message formats on the arm CAN bus.** These are 11-bit CAN 2.0A IDs, COB-ID = function
code + node ID, little-endian payloads. `pxx` configures the PDO layouts at activation.

| frame | COB-ID | dir | payload | when |
|---|---|---|---|---|
| NMT | `0x000` | host → | `[cmd, node]`: 01 start, 02 stop, 80 pre-op, 81 reset (node 0 = all) | activation / shutdown |
| SYNC | `0x080` | host → | empty | every control cycle (100 Hz) |
| RPDO1 | `0x200+id` | host → | int32 target position `607Ah` (counts) | each cycle, once a controller commands |
| TPDO1 | `0x180+id` | → host | int32 position `6064h` + int32 velocity `606Ch` | after every SYNC |
| TPDO2 | `0x280+id` | → host | uint16 status word `6041h` | after every SYNC |
| EMCY | `0x080+id` | → host | uint16 error code (`8611` following error, `8130` heartbeat, …) | on a fault |
| SDO request | `0x600+id` | host → | `[cs, idx lo, idx hi, sub, data×4]`: cs 40 read, 2F/2B/23 write 1/2/4 bytes | configuration only |
| SDO reply | `0x580+id` | → host | same layout: cs 43/4B/4F read data, 60 write ok, 80 abort (+ abort code) | configuration only |
| heartbeat | `0x700+id` | → host | NMT state (00 boot-up, 05 operational, 7F pre-op) | at power-on and on heartbeat period |

**Units:** 65 536 counts per **motor-shaft** revolution. The joint angle is counts ÷ 65 536 ÷
`gear_ratio` × 2π, where `gear_ratio` = `builtin_reduction` × `external_reduction` ×
`direction`.

### 3.2 UART (RoboClaw)

```bash
# with the arm launch STOPPED (only one program can own the UART):
pixi run --environment l4t ros2 run arm_bringup roboclaw_probe.py --port /dev/roboclaw --baud 38400
#   address 128: USB Roboclaw 2x15a v4.x.x
#     main battery  24.1 V
#     temperature   28.3 C
#     currents      M1 0.00 A, M2 0.00 A
#     encoders      M1 0, M2 0
#   answering addresses: [128, 129]
pixi run --environment l4t ros2 run arm_bringup roboclaw_probe.py --addresses 128 --watch
#   turn each joint by hand: the matching encoder must change, in the + direction
```

**Pass:** every `address` in `arm_axes.yaml` answers. Turning each joint by hand changes the
right channel (M1 vs M2), in the direction you expect. A *bad CRC* reply means a baud
mismatch, or two units sharing an address. While the arm launch runs, check RoboClaw axes
on `/joint_states` (position only).

**Packet-serial format** (all multi-byte fields **big-endian**, CRC16-CCITT/XMODEM: poly
`0x1021`, init 0):

| | bytes |
|---|---|
| read request | `[address][command]` |
| read reply | `[data…][CRC hi][CRC lo]`, where the CRC covers address + command + data |
| write request | `[address][command][data…][CRC hi][CRC lo]` |
| write reply | `0xFF` |

Commands in use:

| cmd | name | data | used by |
|---|---|---|---|
| 21 | read firmware version | → ASCII string, `\n\0` | probe |
| 24 | read main battery | → uint16, 0.1 V | probe |
| 49 | read motor currents | → int16 M1, int16 M2, 10 mA | probe |
| 78 | read encoder counters | → int32 M1, int32 M2 | **driver** (every cycle), probe |
| 82 | read temperature | → uint16, 0.1 °C | probe |
| 90 | read status | → status bits | probe |
| 37 | drive M1/M2 with signed speed | ← int32 M1, int32 M2, counts/s | **driver** (every cycle) |

Example: reading the encoders of address 128 sends `80 4E` and gets back
`[M1 ×4][M2 ×4][CRC ×2]`.

---

## 4. Test speeds (very slow)

The bring-up speed is **0.05 rad/s at the joint (≈ 2.9 °/s)**. It's set per axis as
`test_velocity` in `arm_axes.yaml`. Speeds are capped by `limits.max_velocity`.

| step | speed | move |
|---|---|---|
| first motion of a new axis | **0.02 rad/s** (`--speed 0.02`) | ±0.05 rad, out and back |
| range check | 0.05 rad/s (default) | small steps toward each end of travel |
| controller tuning check | 0.05 → 0.2 rad/s | short moves, watch tracking |

```bash
pixi run --environment l4t ros2 run arm_bringup arm_slow_jog.py joint_arm_1 0.05 --back --speed 0.02
pixi run --environment l4t ros2 run arm_bringup arm_slow_jog.py joint_arm_3 -0.1          # test_velocity
```

`arm_slow_jog.py` moves one joint by a relative amount:

- It **refuses targets outside `limits`** and speeds above `max_velocity`.
- P17/P20 axes get a timed trajectory, so the speed is constant.
- RoboClaw axes get a constant velocity for |Δ| / speed seconds, then zero. It stops early
  if the position leaves its limits.
- Ctrl-C cancels the trajectory, or sends zero velocity.

Keep these backstops on during bring-up, in case someone bypasses the tool:

- RoboClaw: low max current, serial timeout.
- P17/P20: lowered `2024h` overspeed threshold.

---

## 5. Launching the arm

```bash
# real arm: P17/P20 on can1, RoboClaws on /dev/roboclaw
pixi run --environment l4t ros2 launch arm_bringup arm.launch.py

# no hardware: pxx mock backend + mock RoboClaw (mock_components/GenericSystem)
pixi run --environment l4t ros2 launch arm_bringup arm.launch.py can_interface:=mock roboclaw_port:=mock

# CAN protocol test against the CANopen emulator (started automatically for vcanN)
~/billee-software-2027/tooling/can-up vcan0
pixi run --environment l4t ros2 launch arm_bringup arm.launch.py can_interface:=vcan0 roboclaw_port:=mock

# one half only
… arm.launch.py roboclaw_port:=none    # CAN axes only
… arm.launch.py can_interface:=none    # RoboClaw axes only
```

| arg | default | values |
|---|---|---|
| `can_interface` | `can1` | `canN` real · `vcanN` virtual + emulator · `mock` · `none` |
| `roboclaw_port` | `/dev/roboclaw` | serial device · `mock` · `none` |
| `roboclaw_baud` | `115200` | must match every RoboClaw's Motion Studio baud |
| `axes_file` | `arm_bringup/config/arm_axes.yaml` | another constants file |
| `controllers_file` | `arm_bringup/config/arm_controllers.yaml` | another controller template |

**What it starts:** everything is generated from `arm_axes.yaml`, so adding or re-gearing an
axis is a YAML edit only.

- **`ros2_control_node`** with up to two hardware systems:
  - **`ArmCan`** runs `pxx/PxxSystemHardware`. Each joint gets `node_id` and the derived
    `gear_ratio`.
  - **`ArmRoboClaw`** runs `roboclaw_hardware_interface/RoboClawHardwareInterface`. Each
    joint gets `address`, `channel`, and the derived `qppr` (counts per joint revolution).
- **Controllers:**
  - `joint_state_broadcaster` → `/joint_states`
  - `arm_can_controller`, a `JointTrajectoryController` with position commands and partial
    goals allowed:
    - action `/arm_can_controller/follow_joint_trajectory`
    - topic `/arm_can_controller/joint_trajectory`
  - `arm_roboclaw_controller`, a `JointGroupVelocityController`: topic
    `/arm_roboclaw_controller/commands`, `Float64MultiArray` in rad/s, with one entry per
    RoboClaw joint in `arm_axes.yaml` order.

There is no arm kinematic model yet, so there's no `robot_state_publisher` or TF. The
generated URDF only carries the `<ros2_control>` blocks.

**Shutdown:** Ctrl-C.

- **P17/P20:** torque is removed (the brake behaviour is still unverified; see the PXX guide
  §3.7), so **support the arm.**
- **RoboClaw:** on a clean Ctrl-C, the hardware is deactivated and the driver commands zero
  speed. If `ros2_control_node` dies instead (`kill -9`, a crash, Jetson power loss), the
  RoboClaws keep the **last speed** until their own serial timeout trips. That's why every
  unit needs one (§1.2).

---

## 6. Physical constants (`arm_axes.yaml`)

`ros2_ws/src/arm_bringup/config/arm_axes.yaml` is the single source of truth. The field
reference is in the file header. Per axis, measure and fill in:

| field | P17 / P20 | RoboClaw | how to measure |
|---|---|---|---|
| `joint` | ✓ | ✓ | Name it to match the future arm URDF (e.g. `shoulder_pitch`). |
| `node_id` / `address`+`channel` | node 1–127 | 128–135, M1/M2 | What you set in §1.2 / PXX guide §3.4. |
| `builtin_reduction` | reducer inside the actuator (P20-90-PRO-121-B = **121**; **P17: unknown**) | gearmotor gearbox ratio | Datasheet, then confirm below. |
| `external_reduction` | belts / gears / harmonic drive after the actuator | same | Count teeth, or measure (below). |
| `encoder_cpr` | — | quadrature counts per motor rev (4 × the encoder's PPR/lines) | Datasheet. |
| `direction` | +1 / −1 | +1 / −1 | Jog +; flip if the joint moves the wrong way. |
| `limits.*` | rad | rad (relative to the pose at launch) | Move by hand to each end and back off a margin. |
| `test_velocity` | 0.05 | 0.05 | §4. |

**Measuring the total ratio** (catches a wrong datasheet value), with motors unpowered or
disabled:

1. Mark the joint, and read the raw count. P17/P20: `pxx_probe.py scan`, `6064h`. RoboClaw:
   `roboclaw_probe.py`, encoders.
2. Turn the **joint** exactly one revolution (or a measured angle θ, scaled to 2π), then
   read the count again.
3. Work out the ratio from the change in counts (Δcounts):
   - **P17/P20:** Δcounts ÷ 65 536 = `builtin_reduction` × `external_reduction`. A
     P20-direct joint gives ≈ 121.
   - **RoboClaw:** Δcounts = `encoder_cpr` × `builtin_reduction` × `external_reduction`,
     which is the `qppr` the launch derives.
4. If it doesn't match the datasheet values, trust the measurement. Fix
   `external_reduction`, or the reducer if the datasheet was wrong.
5. Set `verified: true`.

The launch derives everything else:

- **pxx:** `gear_ratio` = built-in × external × direction.
- **RoboClaw:** `qppr` = round(encoder_cpr × built-in × external) × direction. It warns if
  rounding changes the value, because the RoboClaw driver takes an integer.

---

## Bring-up order (checklist)

1. [ ] `arm.launch.py can_interface:=mock roboclaw_port:=mock` comes up; `arm_slow_jog.py` moves each joint
2. [ ] `arm.launch.py can_interface:=vcan0 roboclaw_port:=mock` + `pxx_probe.py rates --interface vcan0`
3. [ ] RoboClaws configured in Motion Studio (§1.2), each tuned (§2.1), settings recorded
4. [ ] UART wired, udev name, `roboclaw_probe.py` finds every address (§3.2)
5. [ ] Every RoboClaw at 115200 baud with a serial timeout set; first real-hardware run of the vendored driver ([§1.5](#15-roboclaw-driver-notes)), with joints free to move
6. [ ] `can1` up at 1 Mbit/s, node IDs set, `pxx_probe.py scan` finds every node (§3.1)
7. [ ] Total ratio measured for every axis, `arm_axes.yaml` filled in, `verified: true` (§6)
8. [ ] Real launch, arm supported: each joint moved with `--speed 0.02` (±0.05 rad), direction checked
9. [ ] Limits walked out at `test_velocity`, then recorded in `arm_axes.yaml`
