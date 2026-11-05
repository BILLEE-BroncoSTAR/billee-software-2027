# teleop

Gamepad → drive command. `teleop.launch.py` starts `joy_node`, which reads `/dev/input/js<device_id>`
and publishes `/joy`, and `joy_tank_drive`, which publishes a `Twist` on
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
| `device_id` | `0` | joystick index |
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

`sim_gz.launch.py` (with `joy_control:=true`) and `ground_station.launch.py` (with `joystick:=true`)
include this launch.

The callback doesn't bounds-check the axis and button indices. A wrong index for your pad
crashes the node.
