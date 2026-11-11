# BILLEE Arm — P17 / P20 Motor Bring-up Guide

Step-by-step guide for bringing up the arm's P17 / P20 joint motors with the `pxx`
ros2_control driver (`pxx/PxxSystemHardware`). Work through the phases in order.
Each phase tests one more layer than the one before it, so when something breaks you
know which layer caused it.

| phase | needs hardware? | what it proves |
|-------|-----------------|----------------|
| [0. Build check](#0-package-prep-one-time) | no | test launch, emulator and controller config are installed |
| [1. Mock backend](#1-mock-backend-no-can) | no | plugin loads, URDF params parse, controllers claim joints |
| [2. vCAN + emulator](#2-vcan--canopen-emulator) | no | real SocketCAN backend: SDO config, PDO mapping, SYNC loop, fault path |
| [3. Bench: raw CAN](#3-bench-one-motor-raw-can-no-ros) | 1 motor | wiring, bitrate, node ID, units, direction, brake |
| [4. Bench: ROS](#4-bench-ros2_control-on-real-motors) | 1–2 motors | driver against real drives, small moves |
| [5. Integrate](#5-integrate-into-the-robot) | arm | arm joints in `robot_description`, trajectory controller |

Commands use the same conventions as [RUN_GUIDE.md](RUN_GUIDE.md) (`pixi run --environment l4t`
on the rover, `default` on a dev machine). Run every terminal from `ros2_ws/`.

---

## How the driver works (read once)

```
controller (JointGroupPositionController / JointTrajectoryController)
        │  position cmd (rad, at the joint)          ▲ position (rad), velocity (rad/s)
        ▼                                            │
PxxSystemHardware  (src/pxx.cpp)  — URDF params, NaN-safe command handling
        │
MotorBackend  ── "mock" / "none" → MockMotorBackend  (ideal servo, no CAN)
              └─ anything else   → SocketCanMotorBackend (raw SocketCAN, CANopen CiA402)
```

What happens on the CAN bus, per drive (see `SocketCanMotorBackend::configure_node`):

1. **`on_activate`**
   - Sends NMT *pre-operational*, then a fault reset (`6040h = 0x80`).
   - Sets the mode to **Cyclic Synchronous Position** (`6060h = 8`).
   - Remaps PDOs over SDO:
     - **RPDO1** (`0x200+id`): `607Ah` target position, int32
     - **TPDO1** (`0x180+id`): `6064h` position + `606Ch` velocity, 2× int32
     - **TPDO2** (`0x280+id`): `6041h` status word, uint16
     - All three use transmission type 1 (act on every SYNC).
   - Reads `6064h` and writes it to `607Ah`, so the drive holds its current pose when enabled.
   - Sends NMT *start*, then the control-word sequence `0x06 → 0x07 → 0x0F` (operation enabled).
   - Reads back the status word. If the drive didn't enable, activation fails and the error
     includes the `603Fh` error code.
2. **Every control cycle** (`update_rate`, 100 Hz in the test config):
   - `write()` sends one RPDO1 per joint whose command isn't NaN, then one **SYNC** (`0x080`).
   - The drives latch the targets and answer with TPDO1 + TPDO2. An RX thread caches these
     replies.
   - `read()` returns the cached values. If any status word has the fault bit set, `read()`
     returns `ERROR`, and controller_manager then deactivates the hardware.
3. **`on_deactivate`**: control word `0x06` (shutdown, **torque off**) and NMT pre-operational.

Units: `CountsToRad` uses `kPositionCountsPerRev = 65536` (in `include/pxx/can.hpp`). That
is the encoder resolution per **motor-shaft** revolution, before the built-in reducer. The
per-joint `gear_ratio` param is motor-shaft turns per joint turn: the **built-in reducer ×
any external gearing**. For a P20-90-PRO-121-B driving a joint directly, `gear_ratio` =
`121.0` (65 536 × 121 = 7 929 856 counts per joint revolution). A P17 or another reduction
only needs a different `gear_ratio`, not a code change.

Commands are NaN until a controller writes one, so the driver sends nothing and the drives
hold position. **Never seed a position command with 0.0.** That would drive every joint to
0 rad.

---

## 0. Package prep (one-time)

The bench launch file (`launch/pxx_test.launch.py`), its controller config
(`config/pxx_test_controllers.yaml`) and the emulator (`scripts/pxx_vcan_emulator.py`) are
in the package and installed. Build and check:

```bash
pixi run --environment l4t build
pixi run --environment l4t ros2 pkg list | grep -x pxx
pixi run --environment l4t ros2 pkg executables pxx      # → pxx pxx_vcan_emulator.py
```

---

## 1. Mock backend (no CAN)

```bash
pixi run --environment l4t ros2 launch pxx pxx_test.launch.py can_interface:=mock
```

Expect these log lines:

- `initialised 2 joints with mock backend (endpoint='mock').`
- `activated mock backend ...`
- Both spawners report `Configured and activated`.

Check in a second terminal:

```bash
pixi run --environment l4t ros2 control list_hardware_interfaces
#   joint_arm_1/position [available] [claimed]   ← command
#   joint_arm_1/position, joint_arm_1/velocity   ← state
pixi run --environment l4t ros2 topic pub --once /arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.5, -0.25]}"
pixi run --environment l4t ros2 topic echo --once /joint_states
#   position: [0.5, -0.25]  (the mock moves there in one cycle)
```

**Pass:** `/joint_states` follows the command exactly.

---

## 2. vCAN + CANopen emulator

This runs the real `SocketCanMotorBackend` against `pxx_vcan_emulator.py`, a minimal CiA402
drive emulator. The emulator packs its PDOs from whatever mapping the driver configured, so
a mapping bug shows up as wrong numbers.

```bash
~/billee-software-2027/tooling/can-up vcan0
pixi run --environment l4t ros2 launch pxx pxx_test.launch.py can_interface:=vcan0
```

The launch file starts the emulator for nodes 1 and 2 automatically.

Watch the bus in another terminal (needs `can-utils`):

```bash
candump -tz vcan0
```

You should see:

1. **Activation:** NMT `000#8001`, then a burst of SDOs (`601#...` request / `581#60...` ack).
   The same happens for node 2.
2. **Steady state:** at 100 Hz, `080#` (SYNC), followed by `181#` (8 B pos+vel) and `281#`
   (2 B status `27 04` / `27 00`) for each node. RPDOs (`201#`, `202#`) appear only after
   you publish a command.

Move the joints:

```bash
pixi run --environment l4t ros2 topic pub --once /arm_position_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.5, -0.25]}"
```

The emulator slews at 30 motor rev/s by default (`--max-speed`), about 0.25 joint rev/s at
the test launch's `gear_ratio` of 121. So `/joint_states` ramps toward the target over
about 0.3 s instead of jumping.

**Fault path test:** this injects EMCY `0x8611` (*position error too large*) on every node:

```bash
pkill -USR1 -f pxx_vcan_emulator
```

**Pass:** the controller_manager log shows
`backend read failed: node 1 in fault (status word 0x0008, EMCY 0x8611: position error too large)`,
and the hardware component deactivates. Restarting the launch clears the fault, because
activation sends a fault reset.

Clean up: `~/billee-software-2027/tooling/can-up down vcan0`.

---

## 3. Bench: one motor, raw CAN (no ROS)

Do this with **one motor at a time** until every motor has a unique node ID. CANopen drives
probably share a factory-default ID, and two drives with the same ID will garble each
other's SDO replies.

### 3.1 Wiring and power

- **Supply:** P17 / P20 run on **24–48 V DC**. Use a current-limited bench supply for the
  first power-up.
- **CAN:** CAN-H, CAN-L and **a common ground** between the drive and the CAN adapter.
- **Termination:** one **120 Ω resistor at each end** of the bus, two in total. Check it with
  the bus powered down: you should read about 60 Ω between CAN-H and CAN-L.
- **Mounting:** clamp the motor, or leave the output shaft free. Don't connect it to an arm
  link yet.

### 3.2 Choose the bus and bitrate

The drives default to **1 Mbit/s**. Supported rates are 1M, 500k, 250k, 125k, 100k, 50k, 20k
and 10k. The ODESC drivetrain uses `can0` at **500 kbit/s**, and `tooling/can0.service` pins
that.

**Put the arm on its own bus** (a second Jetson CAN controller or a USB-CAN adapter, e.g.
`can1`). Sharing `can0` would mean retuning every drive to 500k. It would also risk ID
collisions: ODrive CANSimple IDs are `(node << 5) | cmd`, so drivetrain nodes 0–5 produce
IDs `0x000–0x0BF`. That range overlaps CANopen NMT (`0x000`), SYNC (`0x080`) and EMCY
(`0x081+`).

```bash
~/billee-software-2027/tooling/can-up can1 1000000
~/billee-software-2027/tooling/can-up status can1     # state UP, bitrate 1000000
```

### 3.3 Find the node ID

Run `candump` first, then power the drive on:

```bash
candump -tz can1
#   7XX#00   ← CANopen boot-up message; node ID = 0xXX - 0x700
```

If you missed the boot-up message, probe with an SDO read of `1000h` (device type) to each
candidate ID. Only the right one answers on `0x580+id`:

```bash
cansend can1 601#4000100000000000      # ask node 1
#   581#43001000xxxxxxxx               ← node 1 is present
```

### 3.4 Assign a node ID and save it

Assignments come from `ros2_ws/src/arm_bringup/config/arm_axes.yaml`. Example: change node 1 to
node 2.

```bash
cansend can1 601#2F01200102000000      # 2001h:01 (servo ID) = 0x02
cansend can1 601#2310100173617665      # 1010h:01 = "save" (0x65766173), stores to flash
# power-cycle the drive, then confirm it boots as 702#00
```

Label each motor with its node ID. Repeat for every drive before you connect more than one
to the bus.

### 3.5 Read the basics

Replace `601`/`581` with `600+id`/`580+id`:

| what | send | reply |
|------|------|-------|
| status word `6041h` | `601#4041600000000000` | `581#4B416000 LL HH 00 00` — `0x0250` (`50 02`) ≈ switch-on disabled |
| error code `603Fh` | `601#403F600000000000` | `581#4B3F6000 LL HH 00 00` — `0000` = no error |
| position `6064h` | `601#4064600000000000` | `581#43646000` + int32 LE |
| gear ratio `6091h` | `601#4091600100000000` | check whether the drive already applies extra scaling |

### 3.6 Verify the unit assumptions

The driver's unit conversion has only been checked against the emulator. Check these on real
hardware **before** commanding motion.

1. **Counts per output revolution.** With the drive disabled, read `6064h`, turn the output
   shaft exactly one revolution by hand (mark it), then read again.
   - The difference should be **65 536 × reducer ratio**: 7 929 856 for a 121:1 P20.
   - Divide the measured counts by 65 536 to get that motor's `gear_ratio`. A whole number
     confirms the 2¹⁶ encoder. If a P17 gives a fractional result, its encoder resolution
     differs, and `kPositionCountsPerRev` has to become a per-joint param.
   - Also read `6091h`. If the drive's own gear ratio is set to scale `6064h` into output
     units, the measured counts will be much lower, and `gear_ratio` must not apply the
     reducer a second time.
2. **Direction.** Note which way the counts increase. If that is opposite to the joint's
   positive direction in the URDF, use a negative `gear_ratio` (e.g. `-121.0`). The driver
   accepts any non-zero finite value.
3. **Velocity units (`606Ch`).** The driver assumes counts/s. You'll check this in Phase 4
   against the derivative of position.
4. **Absolute vs. relative position.** Power-cycle and re-read `6064h` without moving the
   shaft. If the value resets to 0, the encoder is not absolute across power cycles, and the
   arm will need a homing / zeroing step.

### 3.7 Brake

`2014h:01` controls the brake (0 = closed / engaged, 1 = released). **The driver never
writes it.** On the bench, find out whether the drive releases the brake automatically when
it reaches *operation enabled*. Watch for the drive faulting with `0x7121` (stalled) or
`0x8611` while holding against the brake. If the brake does not auto-release, the driver
needs to release it after enable and engage it before shutdown. That is the open TODO in
`SocketCanMotorBackend::deactivate`.

Manual release, **only with the shaft unloaded**:

```bash
cansend can1 601#2F14200101000000      # release
cansend can1 601#2F14200100000000      # engage
```

### 3.8 Heartbeat

Error `0x8130` (*heartbeat offline*) means the drive expected a host heartbeat. The driver
sends SYNC but **no heartbeat**. If drives fault with `8130` a moment after enabling, either
disable consumer heartbeat monitoring on the drive (`1016h`) or add a host heartbeat to the
driver.

---

## 4. Bench: ros2_control on real motors

Edit `JOINTS` in `launch/pxx_test.launch.py` to match the node IDs on the bench. For one
motor, keep a single entry and shorten `joints:` in `config/pxx_test_controllers.yaml` to
match. Rebuild.

```bash
~/billee-software-2027/tooling/can-up can1 1000000
pixi run --environment l4t ros2 launch pxx pxx_test.launch.py can_interface:=can1
```

The launch only starts the emulator for `vcan*` interfaces, so no emulator runs here.

**Pass criteria, in order:**

1. Activation completes with no SDO timeouts or aborts.
2. `candump can1` shows `080#` / `18X#` / `28X#` at the update rate.
3. `/joint_states` position matches the `6064h` reading from 3.5 (after unit conversion),
   and velocity reads ≈0 at rest.
4. **Small move first.** Read the current position from `/joint_states` and command
   **+0.05 rad** from it. In CSP the drive runs to the target "at maximum allowed speed", so
   `JointGroupPositionController` step commands are violent and can trip `0x8611` (following
   error vs `6065h`). Keep steps tiny with this controller.
   ```bash
   pixi run --environment l4t ros2 topic echo --once /joint_states
   pixi run --environment l4t ros2 topic pub --once /arm_position_controller/commands \
     std_msgs/msg/Float64MultiArray "{data: [<current + 0.05>]}"
   ```
5. The direction matches the expected joint direction, and the move size matches (0.05 rad ≈ 2.9°).
6. Velocity during a move ≈ Δposition / Δt. If it's off by a constant factor, fix
   `kVelocityCountsPerRevPerSec` (step 3.6.3).
7. **Ctrl-C:** the drive shuts down cleanly. **Note what the shaft does when torque drops**
   (brake engaged or freewheeling). That decides how the arm has to be supported in Phase 5.

---

## 5. Integrate into the robot

The `arm_bringup` package does this. The full procedure, RoboClaw axes included, is in
[ARM_BRINGUP_GUIDE.md](ARM_BRINGUP_GUIDE.md). In short:

1. **Fill in `arm_bringup/config/arm_axes.yaml`** with the real joint names, motor model,
   node ID, reductions and direction for every joint. It's the single source of truth:
   `arm.launch.py` derives `gear_ratio` and generates the `<ros2_control>` block from it,
   so nothing is transcribed by hand.
2. **Launch** with `ros2 launch arm_bringup arm.launch.py` (`can_interface:=can1` by
   default; `mock` / `vcanN` / `none` work as here). It runs a `joint_trajectory_controller`
   (`arm_can_controller`) on the CAN axes, not `JointGroupPositionController`. JTC
   interpolates between points and seeds its commands from the current state, which works
   well with the driver's NaN-until-commanded behaviour.
3. **Move slowly** with `ros2 run arm_bringup arm_slow_jog.py <joint> <delta> --speed 0.02`.
   It checks the limits from `arm_axes.yaml`.
4. **Persist the bus.** Copy `tooling/can0.service` to a `can1.service` with
   `bitrate 1000000`, or extend `can-up` with an arm default.
5. Do the first full-arm activation with the arm **physically supported**.

---

## Troubleshooting

| symptom | likely cause |
|---------|--------------|
| `CAN interface 'canX' not found` | Interface not up. Run `tooling/can-up canX <bitrate>`. |
| `node N SDO 0x6040:0x00 timed out` (first SDO) | Wrong node ID, wrong bitrate, missing termination / ground, or the drive isn't powered. `candump` should show the boot-up `7XX#00` at power-on. On vcan the emulator may not have started yet, so relaunch. |
| `candump` shows nothing, `ip -s link` shows errors / `BUS-OFF` | Bitrate mismatch or bad termination. `can-up` sets `restart-ms 100`, so the bus recovers once the cause is fixed. |
| `SDO ... aborted: 0x06020000 (object does not exist)` | The drive's OD differs from the manual (check its firmware version), or the PDO slot isn't remappable. |
| `SDO ... aborted: 0x06010002 (write not allowed)` | The object was written in the wrong NMT / CiA402 state. PDO mapping must happen in pre-operational, which the driver ensures. |
| `failed to enable (status word 0x..., error 0x8130)` | Heartbeat expected (see 3.8). |
| `... error 0x7121 / 0x8611` right after enable | Brake still engaged (see 3.7), or target ≠ actual. The driver seeds `607Ah` from `6064h` before enabling. |
| Faults with `0x8611` during moves | Steps too large for the following-error window `6065h`. Use the trajectory controller or smaller steps. |
| Position off by a constant factor | `gear_ratio` wrong for this motor. Off by exactly 121× usually means the reducer is missing or applied twice (see 3.6.1). |
| Joint moves the wrong way | Negate `gear_ratio`. |
| State never updates but activation succeeded | No TPDOs. Check that `candump` shows `18X#` after each `080#`. The TPDO mapping or the NMT start may have been rejected. |
| Arm drops on Ctrl-C / deactivate | Shutdown removes torque. Resolve the brake behaviour (see 3.7) before running unsupported. |

---

## Open items before the arm runs unattended

- [ ] Real joint names and node IDs in `arm_bringup/config/arm_axes.yaml` (currently `joint_arm_1` / `joint_arm_2` placeholders)
- [ ] Confirm the 2¹⁶ counts per motor rev holds for **each** motor model
- [ ] P17 reducer ratio in `arm_axes.yaml` (currently a `121.0` placeholder)
- [ ] Confirm `606Ch` velocity units
- [ ] Confirm `6091h` gear ratio doesn't double-scale `6064h`
- [ ] Brake auto-release behaviour, and brake engage on deactivate
- [ ] Heartbeat requirement (`1016h` / `0x8130`)
- [ ] Absolute position across power cycles, or a homing procedure
- [ ] Soft limits (`2003h`) and/or URDF joint limits set
- [ ] Dedicated arm CAN bus brought up at boot
