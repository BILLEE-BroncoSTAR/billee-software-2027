# status_light

The rover's mode light: a [blink(1)](https://blink1.thingm.com/) USB LED on the Jetson showing
which state of the rover state machine the rover is in.

| State | Light | Pattern name |
|---|---|---|
| Idle (dashboard up, nothing running) | yellow, slowly pulsing | `idle` |
| Teleop · Drive, Teleop · Arm, Teleop · Science | solid blue | `teleop` |
| Autonomous, Astronaut | solid red | `autonomous` |
| Autonomous mode reached its goal (for later) | flashing green | `arrived` |
| | dark | `off` |

## How it runs

- **From a mode:** `rover.launch.py status_light:=teleop` (or `autonomous`, …) starts the node with
  that pattern. The BILLEE dashboard passes the right one for each mode. Each mode's **Mode light**
  setting turns it off.
- **Idle:** while no mode runs, the dashboard keeps
  `ros2 run status_light status_light --ros-args -p pattern:=idle` running on the rover, and stops
  it before a mode starts.
- **By hand:** `ros2 launch status_light status_light.launch.py pattern:=teleop`.
- **Switching at run time:** publish a pattern name on `/status_light/pattern`. The autonomous mode
  will use this to show "arrived":

  ```bash
  ros2 topic pub --once /status_light/pattern std_msgs/msg/String "{data: arrived}"
  ```

  `/status_light/state` (latched) says which pattern is showing.

Parameters: `pattern` (default `idle`), `serial` (pick one blink(1) when several are plugged in),
`off_on_exit` (default `true`: the light goes dark when the node stops).

## Setup

- **Driver:** the PyPI package `blink1` (with `hidapi`), from `ros2_ws/pixi.toml`, so
  `pixi install` brings it in.
- **USB permission:** `docker/blink1_udev.sh` installs `docker/51-blink1.rules` on the Jetson
  host at every start of the l4t dev container (`postStartCommand`), so `ros_user` can open the
  light. It needs no image rebuild: restart the container, or run `sudo sh docker/blink1_udev.sh`
  in it once.
- **No light plugged in** (or no permission): the node keeps running, logs a warning every 30 s
  and retries every 3 s. The light picks the colour up as soon as it's plugged in, so a missing
  light never stops a launch.

## Files

| File | |
|---|---|
| `status_light/patterns.py` | the colours and timings (no ROS, no USB: unit-tested in `test/`) |
| `status_light/node.py` | the node: parameters, topics, reconnecting to the blink(1) |
| `launch/status_light.launch.py` | the node on its own |
