# Running the rover

There are two machines:

| Machine | Dev container | Pixi env | Has |
|---|---|---|---|
| **Jetson** (the rover) | `.devcontainer/l4t/` | `l4t` | CAN bus, ZED 2i. No monitor. |
| **Ground station** (x86 laptop) | `.devcontainer/devcontainer.json` | `default` | Gamepad, screen, Foxglove Studio |

Every command below runs **inside the dev container, in `ros2_ws`, inside `pixi shell -e <env>`**,
on the machine named above it. The only exceptions are marked **host** (outside the container).

- [Before the first run](#before-the-first-run)
- [Pick a setup](#pick-a-setup)
- [Foxglove](#foxglove)
- [Controls](#controls)
- [Launch options](#launch-options)
- [CAN testing without motors](#can-testing-without-motors)
- [People / pose](#people--pose)
- [Troubleshooting](#troubleshooting)

## Before the first run

On **each** machine you use:

```bash
cd ros2_ws
pixi shell -e l4t                 # Jetson.  Ground station: pixi shell -e default
pixi run build                    # 8 GB Jetson: add --parallel-workers 2 --executor sequential
source install/local_setup.bash   # only in shells that were open before the build
```

For two machines to talk:

- Both are on the same LAN, and the network allows multicast. Some Wi-Fi access points block
  traffic between clients.
- Both use `ROS_DOMAIN_ID=42`. Pixi sets it. A shell outside pixi needs `export ROS_DOMAIN_ID=42`.

## Pick a setup

| Setup | Jetson runs | Ground station runs |
|---|---|---|
| [1. Sim on one machine](#1-sim-on-one-machine) | nothing | Gazebo, rover, teleop, Foxglove bridge |
| [2. Sim on the Jetson](#2-sim-on-the-jetson-driven-from-the-ground-station) | Gazebo, rover, Foxglove bridge | teleop, RViz, Foxglove bridge |
| [3. Real drivetrain](#3-real-drivetrain-driven-from-the-ground-station) | ODESC drivetrain, Foxglove bridge | teleop, RViz, Foxglove bridge |
| [4. Bench test](#4-bench-test-everything-on-the-jetson) | drivetrain, teleop, viewers | nothing |

Teleop (the gamepad nodes) must run on exactly **one** machine. If it runs on both, every drive
command is published twice.

### 1. Sim on one machine

**Ground station** (or any x86 machine with a screen and gamepad):

```bash
ros2 launch chassis_bringup sim_gz.launch.py foxglove:=true
```

Then [connect Foxglove](#foxglove) to `ws://localhost:8765`.

### 2. Sim on the Jetson, driven from the ground station

**Jetson:**

```bash
xvfb-run -a vglrun -d egl ros2 launch chassis_bringup rover.launch.py
```

The Jetson has no monitor, but Gazebo needs an X display to render the simulated camera.
`xvfb-run` provides a virtual one, and `vglrun` puts the rendering on the GPU. Wait until the log
shows `diff_drive_controller` active (about 30 s). The sim runs at about 0.33× real time on the
Orin Nano.

**Ground station:**

```bash
ros2 launch chassis_bringup ground_station.launch.py
ros2 topic list | grep diff_drive     # in a second terminal: should list /diff_drive_controller/*
```

Then [connect Foxglove](#foxglove) and drive.

### 3. Real drivetrain, driven from the ground station

**Jetson, host** (once per boot):

```bash
tooling/can-up            # can2 @ 500 kbit/s
tooling/can-up status
```

To bring it up on every boot:
`sudo cp tooling/can2.service /etc/systemd/system/ && sudo systemctl enable --now can2.service`.

**Jetson:**

```bash
ros2 launch chassis_bringup rover.launch.py mode:=real
ros2 control list_controllers     # in a second terminal: both controllers "active"
```

With no motors connected, add `can_interface:=mock`. Commands then loop back as wheel feedback,
and you can skip `can-up`.

**Ground station:**

```bash
ros2 launch chassis_bringup ground_station.launch.py use_sim_time:=false
```

There is no `odom` frame on the real drivetrain (no EKF yet). In RViz, set **Fixed Frame** to
`base_link`. The Foxglove layout already uses `base_link`.

### 4. Bench test, everything on the Jetson

Gamepad and monitor plugged into the Jetson:

```bash
ros2 launch chassis_bringup onboard_test.launch.py                       # real CAN
ros2 launch chassis_bringup onboard_test.launch.py can_interface:=mock   # no motors
```

## Foxglove

Foxglove Studio is **not** in the containers. Install the desktop app on the ground station
**host** from [foxglove.dev/download](https://foxglove.dev/download). The containers use host
networking, so the host can reach their port 8765.

1. **Open connection → Foxglove WebSocket**, then enter the URL:

   | URL | Bridge | Use when |
   |---|---|---|
   | `ws://localhost:8765` | the ground station's own, started by `ground_station.launch.py` (setup 1: `sim_gz.launch.py foxglove:=true`) | normal case. It shows the Jetson's topics, received over DDS. |
   | `ws://<jetson-ip>:8765` | the Jetson's, started by `rover.launch.py` | DDS between the machines doesn't work, or you have no ground-station container. Get the IP with `hostname -I` on the Jetson. |

2. **Layouts → Import from file**, and pick the layout for what is running:

   | Running | Layout |
   |---|---|
   | drivetrain (sim or real) | `ros2_ws/src/chassis_bringup/foxglove/drivetrain.json` |
   | ZED point cloud only | `ros2_ws/src/perception/people/foxglove/zed_points.json` |
   | OpenPose + point cloud | `ros2_ws/src/perception/people/foxglove/people_rover.json` |

**Mac:** run Foxglove on macOS and connect to `ws://localhost:8765`. The Mac container forwards
the port. The Mac container has no GPU and no gamepad, so run the sim as
`xvfb-run -a ros2 launch chassis_bringup sim_gz.launch.py gui:=false foxglove:=true joy_control:=false`
and drive with the keyboard (below).

## Controls

Xbox or PlayStation gamepad (USB `/dev/input/js0`) on the machine that runs teleop:

- **Hold RB / R1** to drive. Release it and the rover stops.
- **Left stick up/down** drives the left wheels, **right stick up/down** the right wheels.

The mapping and speed are in `src/teleop/config/joystick.yaml`. Edits take effect without
rebuilding.

Without a gamepad, from any machine on the ROS graph:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.4}, angular: {z: 0.3}}'
```

## Launch options

| Launch | Option | Default | |
|---|---|---|---|
| `rover.launch.py` | `mode` | `sim` | `sim` (Gazebo) or `real` (ODESC) |
| | `can_interface` | `can2` | `mode:=real` only: `can2`, `vcan0` or `mock` |
| | `joy_control` | `false` | teleop on the Jetson. Only when no ground station runs it. |
| `ground_station.launch.py` | `use_sim_time` | `true` | `false` when the Jetson runs `mode:=real` |
| | `joystick` / `rviz` / `foxglove` | `true` | |
| | `fprime_gds` | `false` | open the F' GDS web UI (`fprime_gds_url`) |
| `sim_gz.launch.py` | `navigation` | `true` | EKF + Nav2. `false` saves CPU but removes the `odom` frame. |
| | `gui` / `foxglove` / `rviz` / `joy_control` | `true` / `false` / `false` / `true` | |

## CAN testing without motors

This tests the real CAN driver against an emulated ODrive on a virtual bus. Create the bus on the
**host** of the machine running the stack:

```bash
tooling/can-up vcan0          # host. Remove later with: tooling/can-up down vcan0
```

Then, in the container, either:

```bash
ros2 launch chassis_bringup sim_gz.launch.py odesc_shadow:=true    # Gazebo + driver + emulator on vcan0

# or, without Gazebo (two terminals):
ros2 run odesc odesc_vcan_emulator.py --interface vcan0
ros2 launch chassis_bringup real.launch.py can_interface:=vcan0
```

`candump -L vcan0` should show six `0x07` frames on start, then `0x0D` frames while driving.

## People / pose

**Jetson**, with the ZED 2i plugged in. Full guide: [people-pose.md](people-pose.md).

```bash
src/perception/zed_openpose/scripts/get_openpose_models.sh   # once, ~300 MB
pixi run build --packages-select people_msgs zed_openpose people robot_description
ros2 launch people people_rover.launch.py                     # OpenPose + point cloud
ros2 launch people people_rover.launch.py openpose:=false     # point cloud only
```

If the drivetrain is already running (setup 3), add `standalone:=false`. View it on the ground
station with the matching [Foxglove layout](#foxglove).

## Troubleshooting

| Symptom | Fix |
|---|---|
| `Package 'chassis_bringup' not found` | The shell was opened before the build. Run `source install/local_setup.bash`. |
| `the task 'build' is ambiguous` | You're outside `pixi shell`. Pass `-e <env>`. |
| Build killed, or the Jetson hangs | Out of RAM: `pixi run build --parallel-workers 2 --executor sequential` |
| Ground station sees no Jetson topics | Check `echo $ROS_DOMAIN_ID` is `42` on both, both are on the same subnet, and nothing blocks multicast or UDP. Foxglove to `ws://<jetson-ip>:8765` still works without DDS. |
| Foxglove "connection refused" | No bridge on that machine, or wrong IP. On that machine, `ss -ltn \| grep 8765` should show it listening. |
| Gamepad connected but the rover doesn't move | Hold RB. Check `ros2 topic echo /joy`, then `ros2 topic echo /diff_drive_controller/cmd_vel_unstamped`. |
| `/joy` is silent | `ls /dev/input/js*`, then set `device_id` in `teleop/config/joystick.yaml`. |
| Every command seems doubled | Teleop runs on both machines. Use `rover.launch.py` (no teleop) on the Jetson, not `tooling/sim-up`. |
| RViz/Foxglove: no `odom` frame | Real drivetrain: expected, use fixed frame `base_link`. Sim: don't pass `navigation:=false`. |
| Sim: `Switch controller timed out`, or `Couldn't open X display` | No usable display. On the Jetson, prefix the launch with `xvfb-run -a vglrun -d egl`. |
| Real: controllers `inactive`, wheels dead | CAN down or wrong bitrate. Run `tooling/can-up` on the host and check `dmesg`. |
| `vcan0` but no frames | Create `vcan0` on the host first, then check with `candump -L vcan0`. |
| `Could not enable FIFO RT scheduling policy` | Harmless. |
