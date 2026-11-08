# Running the rover

All commands run from `ros2_ws` inside `pixi shell -e <env>` (`default` on x86, `l4t` on the
Jetson, `mac-cpu` on a Mac). Build first with `pixi run build` (see the [README](../README.md)).

- [Controls](#controls)
- [Simulation on one machine](#simulation-on-one-machine)
- [Rover + ground station](#rover--ground-station)
- [Real drivetrain](#real-drivetrain)
- [CAN testing without motors](#can-testing-without-motors)
- [People / pose](#people--pose)
- [Troubleshooting](#troubleshooting)

## Controls

Xbox or PlayStation gamepad (USB `/dev/input/js0`), tank drive:

- **Hold RB / R1**: deadman. Release it and the rover stops.
- **Left stick up/down**: left wheels.
- **Right stick up/down**: right wheels.

The mapping and speed scale are in `src/teleop/config/joystick.yaml`. You can edit it without
rebuilding.

Without a gamepad, use the keyboard or a fixed command:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.4}, angular: {z: 0.3}}'
```

## Simulation on one machine

```bash
ros2 launch chassis_bringup sim_gz.launch.py foxglove:=true
```

This starts Gazebo, the rover, `diff_drive_controller`, gamepad teleop, and the navigation
stack (synthetic VIO + EKF, which publishes `odom → base_link`, plus the Nav2 controller and
planner servers). On a slow machine, `navigation:=false` skips the navigation stack, but then
there is no `odom` frame (see [Troubleshooting](#troubleshooting)).

**No monitor** (the rover, SSH): Gazebo needs an X display even without its window, because the
simulated camera is rendered through X. Run it on a virtual display, with the rendering on the GPU:

```bash
xvfb-run -a vglrun -d egl ros2 launch chassis_bringup sim_gz.launch.py gui:=false foxglove:=true
```

`vglrun` (VirtualGL) and `xvfb-run` are in the Jetson image. On the Mac, use `xvfb-run -a` alone
(software rendering). Details: [gazebo.md](gazebo.md#headless-gpu-rendering-jetson).

`tooling/sim-up [foxglove] [joystick] [can_sim]` (each `true`/`false`, default `true true false`)
does all of this from any directory. It picks the pixi env for the machine, and when there is no
working display it switches to `xvfb-run` + `vglrun` with `gui:=false`.

| Arg | Default | |
|---|---|---|
| `gui` | `true` | Gazebo window. `false` = server only (still needs a display, see above) |
| `foxglove` | `false` | Foxglove bridge on `:8765` |
| `rviz` | `false` | RViz with `rviz/drivetrain.rviz` (needs a display) |
| `joy_control` | `true` | gamepad teleop nodes |
| `navigation` | `true` | Synthetic VIO + EKF pose estimate (VIO pose, wheel-odom velocity) and the Nav2 controller/planner servers. The EKF is the only `odom → base_link` TF source, see [below](#troubleshooting). |
| `odesc_shadow` | `false` | Also run the real ODESC driver against `vcan0`, see [CAN testing](#can-testing-without-motors) |
| `x` `y` `z` `yaw` | `0 0 0.2 0` | spawn pose |

**View it in Foxglove:** Open connection → `ws://localhost:8765`, then Layouts → Import from file →
`src/chassis_bringup/foxglove/drivetrain.json`.

**Mac:** there is no gamepad passthrough and no RViz. Run the sim with `xvfb-run -a` and
`joy_control:=false`, drive with keyboard teleop in a second terminal, and use Foxglove on macOS
(the container forwards port 8765).

## Rover + ground station

The rover (Jetson) runs the drivetrain and a Foxglove bridge. The ground station (x86 laptop) runs
the gamepad, RViz and Foxglove. The two machines share a ROS graph over DDS.

**Rover:**

```bash
xvfb-run -a vglrun -d egl ros2 launch chassis_bringup rover.launch.py   # sim backend (Gazebo, no window)
ros2 launch chassis_bringup rover.launch.py mode:=real               # real ODESC drivetrain
ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=mock   # no motors
```

The rover runs no gamepad teleop (`joy_control` defaults to `false`), because the ground station
runs it. If both ran it, each would react to the ground station's `/joy` and every drive command
would be published twice. Pass `joy_control:=true` only when no ground station is involved.

**Ground station:**

```bash
ros2 launch chassis_bringup ground_station.launch.py                       # rover in sim
ros2 launch chassis_bringup ground_station.launch.py use_sim_time:=false   # rover on real hardware
```

`ground_station.launch.py` args: `joystick`, `rviz`, `foxglove` (all `true` by default) and
`fprime_gds` (`false`). `fprime_gds:=true` opens the F' GDS web UI at `fprime_gds_url`
(default `http://192.168.4.73:5000`).

For the two machines to see each other:

- Both use the same `ROS_DOMAIN_ID`. Pixi sets it to `42` (`pixi.toml` → `[activation.env]`).
  A shell outside pixi must `export ROS_DOMAIN_ID=42` itself.
- Both are on the same LAN, with multicast allowed and no host firewall blocking DDS UDP.

Test the link from the ground station before you drive. `ros2 topic list` should show
`/diff_drive_controller/*`, then the `topic pub` command from [Controls](#controls) should move
the rover.

As an alternative to DDS, Foxglove Studio on any laptop can connect straight to the rover's
bridge at `ws://<rover-ip>:8765` (`hostname -I` on the rover).

## Real drivetrain

1. **Bring up the CAN bus** on the Jetson host. The drivetrain's default interface is `can2`:

   ```bash
   tooling/can-up                 # can2 @ 500 kbit/s; idempotent
   tooling/can-up status
   ```

   To bring it up on every boot:
   `sudo cp tooling/can2.service /etc/systemd/system/ && sudo systemctl enable --now can2.service`.
   You can skip this step for `can_interface:=mock`.

2. **Launch:**

   ```bash
   ros2 launch chassis_bringup real.launch.py foxglove:=true
   ```

   | Arg | Default | |
   |---|---|---|
   | `can_interface` | `can2` | real bus, `vcan0` (virtual bus), or `mock` (no CAN; commands loop back as feedback) |
   | `gear_ratio` | `48.0` | motor turns per wheel turn |
   | `foxglove` / `rviz` | `false` | viewers |

   The CAN node ID → wheel map is in the URDF. You don't pass it as an argument (see
   [odesc](../ros2_ws/src/odesc/README.md)).

3. **Verify:**

   ```bash
   ros2 control list_hardware_components   # Robot = active
   ros2 control list_controllers           # diff_drive_controller, joint_state_broadcaster = active
   ```

For a bench test with everything on the rover (drivetrain, teleop, RViz and Foxglove), use
`ros2 launch chassis_bringup onboard_test.launch.py`.

## CAN testing without motors

`vcan0` is a virtual CAN bus in the host kernel. Create it on the **host**, not in the container
(the containers use host networking, so they see it):

```bash
tooling/can-up vcan0          # needs the vcan kernel module
tooling/can-up down vcan0     # remove it when done
```

Then either:

```bash
# Gazebo drives the rover, and the real ODESC driver + a 6-node ODrive emulator run alongside it
ros2 launch chassis_bringup sim_gz.launch.py odesc_shadow:=true    # always uses vcan0

# or: real stack against the emulator only (two terminals)
ros2 run odesc odesc_vcan_emulator.py --interface vcan0
ros2 launch chassis_bringup real.launch.py can_interface:=vcan0
```

Watch the frames with `candump -L vcan0`. You should see six `0x07` (set state) frames on
activate, then `0x0D` (set velocity) frames while driving.

## People / pose

Jetson + ZED 2i only. The full guide is [people-pose.md](people-pose.md).

```bash
src/perception/zed_openpose/scripts/get_openpose_models.sh      # once, ~300 MB
pixi run build --packages-select people_msgs zed_openpose people robot_description
ros2 launch people people_rover.launch.py                        # OpenPose + point cloud
ros2 launch people people_rover.launch.py openpose:=false        # point cloud only
# add standalone:=false next to real.launch.py foxglove:=true
```

On the ground station, import the matching layout: `src/perception/people/foxglove/zed_points.json`
(point cloud) or `people_rover.json` (OpenPose + point cloud). RViz has
`people/rviz/zed_points.rviz` and `people_rover.rviz`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| Build killed or the Jetson hangs | Out of RAM: `pixi run build --parallel-workers 2 --executor sequential` |
| `package 'chassis_bringup' not found` | Not built, or the shell was opened before the build. Run `source install/local_setup.bash`. |
| `the task 'build' is ambiguous` | You're outside `pixi shell`. Pass `-e <env>`. |
| Robot model doesn't move in RViz/Foxglove, or there's no `odom` frame | `diff_drive_controller` doesn't publish `odom → base_link` (`enable_odom_tf: false`). The EKF does, but only in sim with `navigation:=true` (the default). If you passed `navigation:=false`, drop it. The real path has no `odom` TF yet, so use fixed frame `base_link` there. |
| Gamepad connected but the rover doesn't move | Hold RB. Check `ros2 topic echo /joy`, then `ros2 topic echo /diff_drive_controller/cmd_vel_unstamped`. |
| `/joy` is silent | `ls /dev/input/js*`. Set `device_id` in `teleop/config/joystick.yaml`. |
| Ground station sees no rover topics | `ROS_DOMAIN_ID` mismatch (`echo $ROS_DOMAIN_ID` should be `42` on both), a different LAN, or a firewall |
| Foxglove "connection refused" | Bridge not running (`foxglove:=true`) or wrong IP. Check `ss -ltn \| grep 8765` on the rover. |
| Real: controllers `inactive`, wheels dead | CAN interface down or wrong bitrate. Run `tooling/can-up` and check `dmesg`. |
| Real: hangs at "Loading controller 'diff_drive_controller'" | `use_sim_time:=true` with no `/clock`. `real.launch.py` handles this. A hand-rolled `ros2_control_node` needs `use_sim_time:=false`. |
| `vcan0` but no frames | Create `vcan0` on the host first, then check with `candump -L vcan0` |
| Gazebo window never appears | Headless. Use `xvfb-run -a vglrun -d egl ... gui:=false` and view in Foxglove. |
| Sim: `Switch controller timed out`, or `OGRE EXCEPTION ... Couldn't open X display` | No usable X display, so the camera renderer crashed and Gazebo stopped stepping. Run under `xvfb-run -a vglrun -d egl` (or `tooling/sim-up`). A stale `$DISPLAY` counts: check with `xdpyinfo`. |
| Sim runs at ~0.33× real time on the Jetson | Expected on the Orin Nano. Gazebo waits for each 10 Hz camera frame, and that work is CPU-bound in the server. See [gazebo.md](gazebo.md#headless-gpu-rendering-jetson). |
| `Could not enable FIFO RT scheduling policy` | Harmless. |
