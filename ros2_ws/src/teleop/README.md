# teleop

Gamepad → drive command. `teleop.launch.py` starts `joy_node`, which opens gamepad `device_id`
(SDL's index, listed by `ros2 run joy joy_enumerate_devices`) and publishes `/joy`, and `joy_tank_drive`, which publishes a `Twist` on
`/diff_drive_controller/cmd_vel_unstamped`.

It is a tank drive gated by a deadman button:

```text
if buttons[safety_button] held:
    linear.x  = (left + right) / 2 * max_vel
    angular.z = (right - left) / track_width * max_vel
else:
    zero Twist
```

It publishes one command per `/joy` message. `joy_node` repeats the last state at 20 Hz.

Parameters (`config/joystick.yaml`, which you can edit without rebuilding):

| Param | Value | |
|---|---|---|
| `device_id` | `0` | SDL joystick index (not the `N` of `/dev/input/jsN`) |
| `deadzone` | `0.05` | |
| `left_axis` | `1` | left stick Y |
| `right_axis` | `4` | right stick Y |
| `safety_button` | `5` (code default) | RB / R1 |
| `max_vel` | `2.0` | scales both linear and angular output |
| `track_width` | `0.67` | m |

```bash
ros2 launch teleop teleop.launch.py
ros2 topic echo /joy                                     # find axis/button indices for a new pad
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped
```

These launches include it:

- `sim_gz.launch.py`, with `joy_control:=true` (the default)
- `ground_station.launch.py`, with `joystick:=true` (the default)
- `rover.launch.py`, only with `joy_control:=true` (default `false`, because the ground station
  runs teleop and two `joy_tank_drive` nodes would publish every command twice)
- `onboard_test.launch.py`, with `joy_control:=true` (the default), through `rover.launch.py`

The callback doesn't bounds-check the axis and button indices. A wrong index for your pad
crashes the node.
