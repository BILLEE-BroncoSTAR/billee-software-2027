# ODESC `ENOBUFS` Diagnostic Plan

## Goal

Identify why the active `odesc/OdescSystemHardware` driver receives
`ENOBUFS` (`No buffer space available`) while transmitting ODrive CANSimple
velocity commands on the 500 kbit/s drivetrain bus.

`ENOBUFS` means the Linux CAN transmit path could not queue another frame. It
does **not**, by itself, prove that `diff_drive_controller` is publishing too
quickly.

## Historical baseline from the recorded bag

The bag in commit `92265f8` (`origin/L4T-Jetson-Setup`),
`ros2_ws/rosbag2_2026_10_25-09_37_04`, recorded a run of the same
`odesc/OdescSystemHardware` source as `main`. The ODESC source and controller
manager update-rate setting are unchanged between that commit and `main`.

Its embedded robot description selected `odesc/OdescSystemHardware` with six
node IDs (`0` through `5`), gear ratio `48.0`, and interface `can3`. Over about
17.2 seconds it recorded:

| Topic | Observed rate |
|---|---:|
| `/joint_states` | 30.05 Hz |
| `/dynamic_joint_states` | 30.03 Hz |
| `/diff_drive_controller/odom` | 30.00 Hz |
| `/diff_drive_controller/cmd_vel_unstamped` | 37.66 Hz |

This is evidence that the current ODESC code path can run at the expected
30 Hz. The bag has no raw CAN frames or interface counters, so it cannot prove
the precise wire load or diagnose the current `ENOBUFS`; it makes an ODESC
source-code regression less likely. The main environmental difference visible
in the bag is its use of `can3`, while `main` defaults to `can0`.

## Safety and setup

- Put the rover on stands or otherwise prevent wheel motion.
- Keep the physical E-stop accessible.
- Use the actual physical interface below. `main` defaults to `can0`; substitute
  `can2` if that is the interface connected to the rover.
- Do not run a second motor-commanding program while the ROS driver is active.
- Record the exact timestamp and full ROS log line for the first `ENOBUFS`.

```bash
CAN_IF=can0
```

## 1. Confirm the runtime stack

The checked-in YAML requests 30 Hz, but the running controller manager is the
authority.

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 param get /controller_manager update_rate
```

Expected:

- Hardware component is active.
- `diff_drive_controller` is active.
- `joint_state_broadcaster` is active.
- `update_rate` is `30`.

If the runtime rate is substantially higher than 30 Hz, stop here and record
the value. The ODESC driver sends six velocity frames per manager update:

```text
TX frames/s = update_rate × 6
```

## 2. Inspect CAN interface state and error counters

Run this once before reproducing the fault and once immediately afterward.

```bash
ip -details -statistics link show "$CAN_IF"
```

Record:

- CAN state: `ERROR-ACTIVE`, `ERROR-WARNING`, `ERROR-PASSIVE`, or `BUS-OFF`.
- TX/RX packet, byte, dropped, and error counters.
- Restart count and configured bitrate.

Interpretation:

| Observation | Meaning / next action |
|---|---|
| `ERROR-ACTIVE`, no growing errors | Continue to traffic-rate checks. |
| `ERROR-PASSIVE` or `BUS-OFF` | Treat this as a CAN physical-layer/bitrate/termination problem first. |
| TX errors grow with each failed driver write | Frames are not draining from the NIC/bus; capture error frames next. |
| RX drops grow | Investigate receive load or socket processing, not command rate alone. |

## 3. Capture CAN error frames during reproduction

Start this *before* launching or commanding the ROS stack. It only listens;
it does not transmit.

```bash
candump -e -L "$CAN_IF"
```

Reproduce `ENOBUFS`, then save the output. Look for `ERRORFRAME`, ACK errors,
bit errors, form errors, CRC errors, overload errors, or bus-off transitions.

If error frames appear, record which events occur immediately before the first
`ENOBUFS`.

## 4. Verify the actual command cadence on the wire

For the current node map `0` through `5`, `Set_Input_Vel` (`cmd 0x0D`) uses
these CAN IDs:

```text
node 0: 00D   node 1: 02D   node 2: 04D
node 3: 06D   node 4: 08D   node 5: 0AD
```

In another terminal, capture normal traffic for several seconds:

```bash
candump -L "$CAN_IF"
```

Expected with the `main` configuration:

- Six `Set_Input_Vel` frames appear as a short burst.
- The burst repeats about every 33 ms (30 Hz).
- The six-frame burst continues even when commanded speed is zero; this is the
  current driver design.

Escalate if:

- Bursts repeat much faster than 30 Hz.
- More than one process is emitting the same `0x0D` IDs.
- Frames stop being visible just before the reported `ENOBUFS`.

## 5. Measure baseline versus ROS traffic

Repeat the interface-statistics command in these three states, preserving each
output:

1. CAN interface up; ROS ODESC/controller-manager stopped.
2. ROS stack active; `diff_drive_controller` inactive.
3. ROS stack active; `diff_drive_controller` active, first at zero command and
   then at a small command with wheels safely elevated.

```bash
ip -details -statistics link show "$CAN_IF"
```

This separates an interface that is already unhealthy from one that fails only
when the hardware `write()` loop begins.

## 6. Measure cyclic encoder-estimate traffic explicitly

The ODESC driver does not configure ODrive telemetry rates; it only consumes
cyclic `Get_Encoder_Estimates` (`cmd 0x09`) frames that each ODrive has been
configured to transmit. Consequently, a saved ODrive setting can overload the
bus without any change to ROS source code. For ODrive firmware 0.5.x, this
persistent setting is `axis0.config.can.encoder_rate_ms`.

The checked-in `configs/odesc/motor_*.json` profiles do **not** provision this
field. Therefore, do not assume cyclic encoder telemetry is enabled or fast
enough to cause `ENOBUFS`; measure it on the live bus before treating it as a
cause. If these profiles are the full source of truth, missing telemetry is a
feedback-configuration gap, not an explanation for transmit-queue exhaustion.

For node IDs `0` through `5`, encoder-estimate IDs are:

```text
009  029  049  069  089  0A9
```

Capture them while all six axes are active:

```bash
candump -L "$CAN_IF",009:7FF,029:7FF,049:7FF,069:7FF,089:7FF,0A9:7FF
```

Count the messages per axis over one second. A configured
`axis0.config.can.encoder_rate_ms` of `10` produces about 100 estimate frames/s
per axis (600 frames/s total), which is comfortable at 500 kbit/s. A 1 ms rate
produces 1,000 frames/s per axis (6,000 total); six standard eight-byte frames
at that rate exceed a 500 kbit/s CAN bus before command or heartbeat traffic is
included. A 2 ms rate is already close to practical saturation.

If the rate is high, reduce and save the ODrive cyclic encoder-estimate setting
consistently on all active axes before changing the ROS driver.

## 7. Check for competing CAN writers

The new ODESC driver must be the only steady motor-command source. Inspect
running processes and launch terminals for:

- a prior `odrive_can_node` instance;
- `cansend` loops or test scripts;
- another `ros2_control_node` / ODESC shadow stack connected to the physical
  interface rather than `vcan0`;
- vendor configuration tools with active CAN traffic.

Also verify the launched interface explicitly, rather than relying on a
default:

```bash
ros2 param list /controller_manager | grep -i robot_description
ip -details link show "$CAN_IF"
```

## 8. Compare with last year's known-good command path

Last year's `ODriveCanNode` sent `Set_Input_Vel` only when its subscribed
`JointState` callback fired. The new ODESC driver sends six frames every
controller-manager tick.

Do **not** run both command paths simultaneously. With the rover safe, compare
them in separate runs:

| Test | What to record |
|---|---|
| Old driver, one axis | CAN state/counters and `0x0D` frame rate. |
| Old driver, all intended axes | Same, plus whether traffic bursts. |
| New ODESC driver at zero command | Same counters and full six-axis cadence. |
| New ODESC driver at low command | First point at which errors or `ENOBUFS` appear. |

The CAN payload format is the same in both implementations: standard CAN ID
`(node_id << 5) | 0x0D`, eight bytes, little-endian float velocity followed by
zero torque feed-forward. This comparison isolates scheduling and system
configuration rather than protocol encoding.

## 9. Decision table

| Result | Most likely cause | Next step |
|---|---|---|
| Runtime update rate is unexpectedly high | ROS configuration/parameter override | Correct the launch/config source, then repeat steps 2–4. |
| CAN error frames or error-passive/bus-off state | Wiring, termination, bitrate, transceiver, or electrical noise | Diagnose the physical CAN bus before modifying ROS code. |
| Encoder estimates are near 1,000 Hz per axis or otherwise dominate the bus | ODrive cyclic telemetry is configured too quickly | Reduce `encoder_rate_ms`, save each controller config, and remeasure. |
| Error-free bus, but `0x0D` rate exceeds expected 180 frames/s | Competing writer or unexpected update loop | Identify and stop the extra writer; verify launch graph. |
| Error-free bus, correct 30 Hz cadence, `ENOBUFS` persists | Socket/NIC queue behavior or driver implementation issue | Add temporary ODESC instrumentation: log frame ID, `errno`, manager cycle count, and `ip` counters at each failure. |
| Old path works at equivalent six-axis rate; new path fails | Regression in current ODESC launch/driver behavior | Compare process count, interface choice, timing, and the current transport send path. |

## 10. Only after evidence: bounded-rate experiment

If steps 1–9 show a healthy, error-free bus but the new driver alone still
causes `ENOBUFS`, make a temporary diagnostic build that transmits only when a
command changes or at a bounded refresh rate (for example 10–30 Hz). Preserve
a periodic refresh if an ODrive CAN watchdog is enabled.

This is a diagnostic experiment, not the first fix: reducing traffic can hide a
physical CAN fault or an unexpected second writer.
