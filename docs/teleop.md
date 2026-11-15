# Gamepad teleop (`teleop`)

Code: [`ros2_ws/src/teleop/`](../ros2_ws/src/teleop) (`src/joy_teleop.cpp`, `config/joystick.yaml`, `launch/teleop.launch.py`).

Gamepad → drive command. `teleop.launch.py` starts `joy_node`, which opens gamepad `device_id`
(SDL's index, listed by `ros2 run joy joy_enumerate_devices`) and publishes `/joy`, and `joy_tank_drive`, which publishes a `Twist` on
`/diff_drive_controller/cmd_vel_unstamped`.

It drives in one of two schemes (`scheme`), both gated by a deadman button:

```text
if buttons[deadman_button] held:
    arcade:  linear.x  = (RT - LT) * max_vel                    triggers 0..1
             angular.z = left_stick_x * steer_scale
    tank:    linear.x  = (left + right) / 2 * max_vel           stick Y -1..1
             angular.z = (right - left) / track_width * max_vel
else:
    zero Twist
```

`arcade` is the default. Pick per launch with `drive_scheme:=arcade|tank` on
`teleop.launch.py` or `ground_station.launch.py`, or set `scheme` in the yaml. A trigger reads as
released until it is first pulled (`joy_node` reports `0.0` for an untouched trigger).

It publishes one command per `/joy` message. `joy_node` repeats the last state at 20 Hz.

Parameters (`config/joystick.yaml`, which you can edit without rebuilding):

| Param | Value | |
|---|---|---|
| `device_id` | `0` | SDL joystick index (not the `N` of `/dev/input/jsN`) |
| `deadzone` | `0.15` | stick travel ignored around centre; the Xbox One S pad drifts ~0.09 at rest |
| `scheme` | `arcade` | `arcade` or `tank` |
| `deadman_button` | `4` | LB / L1, hold to drive (RB / R1, button 5, broke on the Xbox One S pad) |
| `max_vel` | `1.0` | top speed in m/s at full trigger (arcade) or full stick (tank). The drivetrain tops out near 1.36 m/s |
| `steer_axis`, `steer_scale`, `invert_steer` | `0`, `1.5`, `false` | arcade: left stick X, rad/s at full stick |
| `throttle_axis`, `reverse_axis` | `5`, `2` | arcade: RT forward, LT reverse |
| `trigger_rest`, `trigger_press` | `1.0`, `-1.0` | arcade: `/joy` value of a released / fully pressed trigger |
| `left_axis`, `right_axis` | `1`, `4` | tank: left stick Y, right stick Y |
| `track_width` | `0.67` | tank: m; scales the turn rate |

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

An axis or button index your pad doesn't have logs a warning every 2 s and publishes zeros
(the rover stays stopped) instead of crashing the node.
