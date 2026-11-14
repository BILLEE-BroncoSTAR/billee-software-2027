# ODESC Driver Completion and vCAN Validation Plan

## Purpose

This document records the remaining work for the new velocity-only ODESC
`ros2_control` driver and the shortest useful vCAN test path.

The immediate goals are:

1. prove that `diff_drive_controller` produces correctly encoded ODrive
   CANSimple velocity commands; and
2. prove that the physical rover can move after its CAN configuration and
   wheel mapping are confirmed.

vCAN proves the ROS-to-SocketCAN protocol path. It does **not** prove a real
ODESC entered closed loop or that a physical wheel turns in the expected
direction.

## Driver status

The new plugin is selected by the existing URDF plugin name,
`odesc/OdescSystemHardware`, and the plugin XML maps that name to
`odesc_ros2_control::OdescHardwareInterface`.

The driver exports the interfaces required by `diff_drive_controller`:

- one `velocity` command interface per wheel;
- `position` and `velocity` state interfaces per wheel.

Wheel commands are converted from ROS wheel rad/s to ODrive motor turns/s:

```text
motor_turns_per_s = wheel_rad_per_s * gear_ratio / (2 * pi)
```

The reverse conversion is applied to encoder feedback. Per-joint `invert`
defaults to `false`; set it to `true` only after a wheels-up direction test
proves that a specific axis has the opposite physical sign.

```xml
<joint name="joint_wheel_l1">
  <!-- existing command/state interfaces -->
  <param name="node_id">2</param>
  <param name="invert">true</param> <!-- only if this wheel proves reversed -->
</joint>
```

## Remaining work

### Required before testing

1. **Build the new raw SocketCAN transport.**

   `src/socket_can.cpp` and `src/epoll_event_loop.cpp` are part of the CMake
   target. Build the package in the ROS environment and correct any compiler
   or linker errors before attempting vCAN or hardware.

2. **Reconcile the physical ODESC configuration with ROS.**

   The current URDF/node map uses node IDs `0` through `5`, while the checked-in
   ODESC JSON profiles use IDs `2` through `7`. Establish the actual physical
   wheel-to-controller map and make the ODESC profiles, URDF, node map, and vCAN
   emulator agree before driving.

   The profiles also currently show `can.protocol: 0`. The deployed ODESC
   configuration must enable ODrive CANSimple (`PROTOCOL_SIMPLE = 1`) for this
   driver to work.

3. **Confirm CANSimple definitions match installed firmware.**

   The legacy transport emitted a four-byte `Set_Axis_Requested_State` payload,
   while the new generated message definition declares eight bytes. Determine
   the installed ODESC firmware version and use one authoritative definition
   set before relying on the physical enable/idle sequence.

4. **Harden SocketCAN failure and cleanup paths.**

   In `socket_can.cpp`, prevent cleanup after failed initialization from
   dereferencing an unregistered event or closing file descriptor 0. Replace
   unbounded interface-name copying with bounded copying. Report `errno` for
   read/send failures.

5. **Validate node IDs during driver initialization.**

   Reject IDs outside `0..63` and duplicate IDs. A standard CANSimple ID uses
   six node-ID bits and five command-ID bits.

### Required for feedback and usable odometry

6. **Enable and verify cyclic encoder estimates.**

   The driver consumes cyclic `Get_Encoder_Estimates` frames. Configure a
   conservative and uniform encoder-report rate on the actual controllers
   (for example, 10 ms after firmware confirmation), then verify all six axes
   transmit. Add stale-feedback detection so ROS cannot silently use old state.

7. **Observe real ODESC state and faults.**

   A successful Linux socket write only proves the frame was queued locally.
   Before field use, consume or inspect ODrive heartbeat/error traffic to prove
   every axis entered closed loop and remains fault-free.

### Deferred / cleanup

- **Command refresh and watchdog:** deliberately deferred. The driver sends a
  velocity frame when the command changes; the checked-in profiles disable the
  watchdog. If a CAN watchdog is enabled later, add a bounded periodic refresh
  first.
- **Mock mode:** `can_interface:=mock` is documented by older launch files but
  is not implemented by the raw SocketCAN driver. It is not needed for vCAN.
  If restored later, prefer an explicit Xacro/launch switch to
  `mock_components/GenericSystem` with `calculate_dynamics:=true`, rather than
  putting mock branches into `odesc.cpp`.
- **Legacy code/docs:** remove or update the old backend header and documentation
  once the raw driver is the sole supported implementation.

## vCAN protocol validation

### What this test proves

With the supplied `odesc_vcan_emulator.py`, this test proves:

```text
/cmd_vel_unstamped
  -> diff_drive_controller
  -> wheel velocity command [rad/s]
  -> ODESC driver conversion [motor turns/s]
  -> SocketCAN frame ID and Set_Input_Vel payload
  -> emulator encoder estimate frame
  -> driver wheel state / joint state / wheel odometry
```

The emulator currently acts on `Set_Input_Vel` (`0x0D`) and emits encoder
estimates (`0x09`). It does not validate that a real ODESC accepts lifecycle
commands such as clear errors, controller mode, or closed-loop state request.

### Setup

Run all processes in the same Linux network namespace.

```bash
# From the repository root; requires sudo.
tooling/can-up vcan0
```

Terminal 1:

```bash
ros2 run odesc odesc_vcan_emulator.py --interface vcan0
```

Terminal 2:

```bash
ros2 launch chassis_bringup real.launch.py can_interface:=vcan0
```

Terminal 3:

```bash
candump -L vcan0
```

### Exercise the controller

After the controller manager, `diff_drive_controller`, and
`joint_state_broadcaster` are active, issue a small command:

```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist '{linear: {x: 0.10}, angular: {z: 0.0}}'
```

Expected observations:

- command frames use IDs `(node_id << 5) | 0x0D`;
- with node IDs `0..5`, their IDs are `00D`, `02D`, `04D`, `06D`, `08D`, and
  `0AD`;
- the input-velocity payload is a little-endian IEEE-754 float in motor
  turns/s, followed by zero torque feed-forward;
- encoder-estimate frames return on IDs `009`, `029`, `049`, `069`, `089`, and
  `0A9`;
- `/joint_states` velocity changes and wheel position advances;
- `/diff_drive_controller/odom` advances while the command is active.

Inspect controller and hardware state as well:

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic echo --once /joint_states
ros2 topic echo --once /diff_drive_controller/odom
```

### Teardown

Stop the ROS launch and emulator, then remove the virtual interface:

```bash
tooling/can-up down vcan0
```

## Physical-rover proof after vCAN

1. Put the rover on stands, maintain E-stop access, and run only one CAN command
   source.
2. Confirm each ODESC's CANSimple protocol, node ID, controller mode, input mode,
   heartbeat, and encoder-report settings.
3. Start the same real launch using the physical interface, then command a very
   small forward velocity.
4. Verify each wheel's direction; add the per-joint `invert` parameter only to
   demonstrated reversed axes.
5. Verify encoder feedback and CAN error counters before a ground-driving test.
