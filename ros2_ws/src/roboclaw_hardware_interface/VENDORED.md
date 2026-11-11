# Vendored: roboclaw_hardware_interface

| | |
|---|---|
| upstream | https://github.com/dumbotics/roboclaw_hardware_interface |
| commit | `ae009807f750ef607a1fd3acdfbb3c047c168e11` |
| license | Apache-2.0 (see `LICENSE`), Copyright Eric Cox 2023 |
| vendored | 2026-11-11, replacing the git submodule. Upstream's `.github/` CI was dropped. |

This is a normal directory in our repo now: edit and commit it like any other package.
Local changes are marked `BILLEE:` in the source.

## Local changes

- **`baud_rate` hardware param** (optional, default 38400) passed to
  `roboclaw_serial::SerialDevice`. `arm_bringup/arm.launch.py` sets it from its
  `roboclaw_baud` arg (default 115200).
- **Fail on init if the port can't be opened.** Upstream carried on with a closed port and
  failed on first use.
- **Serial errors no longer crash `ros2_control_node`.** `read()`/`write()` catch
  per-unit exceptions, log a warning and skip that cycle. Only 5 failures in a row (50 ms at
  100 Hz) return `ERROR`, which makes controller_manager deactivate the hardware. Upstream
  let the exception escape, which killed every hardware system in the process, including
  the arm's CAN axes.
- **Base `on_init` is now called.** Upstream skipped `SystemInterface::on_init(info)`, so the
  component had an empty name (logged as hardware `''`), and
  `ros2 control set_hardware_component_state` couldn't address it.
- **Zero speed on deactivate** (`on_deactivate`). Upstream left the last speed command
  running on the RoboClaws. On a clean shutdown (Ctrl-C), controller_manager deactivates the
  hardware, so the motors now stop.
- **No driving unless ACTIVE.** Humble keeps calling `write()` on INACTIVE hardware, with
  whatever a still-active controller left in the command interfaces. `write()` now sends
  zero speed unless the hardware is ACTIVE.
- Address-range error message corrected to [128:135].

None of this helps if `ros2_control_node` is killed (`kill -9`, a crash, a Jetson power
loss): the RoboClaws keep the last speed. **Set each RoboClaw's serial timeout** (Motion
Studio, or packet command 14) so they stop on their own.

### How these were tested

- `roboclaw_serial` unit tests: 8 passing (2 upstream, 6 new).
- End to end: the real plugin, via `arm_bringup/arm.launch.py`, against a fake RoboClaw on a
  pseudo-terminal that sends replies one byte at a time.
  - A 0.1 rad jog moved exactly the expected 76 counts.
  - Dropping every 50th reply only produced warnings.
  - Ctrl-C and a lifecycle deactivate both stopped a running motor.
- **Not yet tested on real RoboClaw hardware.**

Still as upstream: velocity command interface and position state only, with position
**relative to activation** (0 rad at startup). There's no mock mode;
`arm_bringup/arm.launch.py roboclaw_port:=mock` substitutes `mock_components/GenericSystem`.
