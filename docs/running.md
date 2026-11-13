# Running the rover

There are two machines:

| Machine | Dev container | Pixi env | Has |
|---|---|---|---|
| **Jetson** (the rover) | `.devcontainer/l4t/` | `l4t` | CAN bus, ZED 2i. No monitor. |
| **Ground station** (x86 laptop) | `.devcontainer/devcontainer.json` | `default` | Gamepad, screen, Foxglove Studio |

Every command below runs **inside the dev container, in `ros2_ws`, inside `pixi shell -e <env>`**,
on the machine named above it. The only exceptions are marked **host** (outside the container).

- [First-time setup](#first-time-setup)
- [Pick a setup](#pick-a-setup)
- [Foxglove](#foxglove)
- [Controls](#controls)
- [Launch options](#launch-options)
- [CAN testing without motors](#can-testing-without-motors)
- [Troubleshooting](#troubleshooting)

## First-time setup

### Jetson

1. **Host**, once per JetPack flash: Docker with the `nvidia` runtime
   ([README §1](../README.md#1-open-the-dev-container)).
2. **Host:**
   ```bash
   git clone https://github.com/BILLEE-BroncoSTAR/billee-software-2027.git ~/billee-software-2027
   ```
3. **Host:** open `~/billee-software-2027` in VS Code (Remote-SSH from a laptop works), then
   **Dev Containers: Reopen in Container** → `rover-roshumble_l4t-aarch64`. Building the image
   compiles OpenPose: 21 min on the Orin Nano, plus the download of the 12.6 GB base image the
   first time.

   Every container start then runs `postStartCommand` (about 1 min the first time): GPU access
   for `ros_user`, the ZED udev rule on the host (`docker/zed_udev.sh`), the ZED's calibration from
   `configs/zed/`, and `pixi install`.
4. **Jetson container:**
   ```bash
   cd ros2_ws
   pixi shell -e l4t                                            # ignore "Could not find activation scripts" before the first build
   pixi run build --parallel-workers 2 --executor sequential    # ~4 min
   source install/local_setup.bash
   src/perception/zed_openpose/scripts/get_openpose_models.sh   # OpenPose weights, ~300 MB
   ```
5. Plug the ZED 2i into a **USB 3** port. On the **host**, `lsusb | grep 2b03` must list
   `STEREOLABS ZED 2i`.

### Ground station

1. **Host:** the x86 requirements in [README §1](../README.md#1-open-the-dev-container), and
   [Foxglove Studio](https://foxglove.dev/download) (the desktop app; it is not in the container).
2. **Host:** clone the repo as above, open it in VS Code, then **Dev Containers: Reopen in
   Container** → `desktop-roshumble_dev-x862`.
3. **Ground-station container:**
   ```bash
   cd ros2_ws
   pixi shell -e default
   pixi run build
   source install/local_setup.bash
   ```
4. Plug the gamepad into the ground station.

### Network

- Both machines are on the same LAN, and the network allows multicast. Some Wi-Fi access points
  block traffic between clients.
- Both use `ROS_DOMAIN_ID=42`. Pixi sets it. A shell outside pixi needs `export ROS_DOMAIN_ID=42`.

### Every new terminal

```bash
cd ros2_ws
pixi shell -e l4t        # Jetson. Ground station: pixi shell -e default
```

## Pick a setup

| Setup | Jetson runs | Ground station runs |
|---|---|---|
| [1. Sim on one machine](#1-sim-on-one-machine) | nothing | Gazebo, rover, teleop, Foxglove bridge |
| [2. Sim on the Jetson](#2-sim-on-the-jetson-driven-from-the-ground-station) | Gazebo, rover, optional ZED point cloud and/or OpenPose, Foxglove bridge | teleop, RViz, Foxglove bridge |
| [3. The rover](#3-the-rover-real-drivetrain-optional-camera-driven-from-the-ground-station) | ODESC (or mock) drivetrain, optional ZED point cloud and/or OpenPose, Foxglove bridge | teleop, RViz, Foxglove bridge |
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

The rover is simulated (Gazebo); the ZED 2i is real. With a camera mode, the real point cloud and
skeletons are drawn on the simulated rover.

**Jetson container**, one of:

| Runs | Command |
|---|---|
| sim | `xvfb-run -a vglrun -d egl ros2 launch chassis_bringup rover.launch.py` |
| sim + ZED point cloud | `xvfb-run -a vglrun -d egl ros2 launch chassis_bringup rover.launch.py camera:=cloud` |
| sim + OpenPose | `xvfb-run -a vglrun -d egl ros2 launch chassis_bringup rover.launch.py camera:=openpose` |
| sim + ZED point cloud + OpenPose | `xvfb-run -a vglrun -d egl ros2 launch chassis_bringup rover.launch.py camera:=both` |

`xvfb-run` gives Gazebo the X display it needs without a monitor; `vglrun` renders on the GPU. The
drivetrain is ready when the log shows `diff_drive_controller` active (about 30 s). The sim runs
at about 0.33× real time.

**Ground-station container**, the line matching the Jetson's `camera:=`:

| Jetson `camera:=` | Command |
|---|---|
| none (default) | `ros2 launch chassis_bringup ground_station.launch.py` |
| `cloud` | `ros2 launch chassis_bringup ground_station.launch.py view:=cloud` |
| `openpose` | `ros2 launch chassis_bringup ground_station.launch.py view:=openpose` |
| `both` | `ros2 launch chassis_bringup ground_station.launch.py view:=both` |

Then [connect Foxglove](#foxglove) and drive.

### 3. The rover: real drivetrain, optional camera, driven from the ground station

**Jetson host**, once per boot, only with the ODESCs connected:

```bash
cd ~/billee-software-2027
tooling/can-up            # can2 @ 500 kbit/s
tooling/can-up status
```

To bring it up on every boot instead:
`sudo cp tooling/can2.service /etc/systemd/system/ && sudo systemctl enable --now can2.service`.

**Jetson container**, one of:

| Drivetrain | Camera | Command |
|---|---|---|
| ODESCs on `can2` | none | `ros2 launch chassis_bringup rover.launch.py mode:=real` |
| ODESCs on `can2` | point cloud | `ros2 launch chassis_bringup rover.launch.py mode:=real camera:=cloud` |
| ODESCs on `can2` | OpenPose | `ros2 launch chassis_bringup rover.launch.py mode:=real camera:=openpose` |
| ODESCs on `can2` | point cloud + OpenPose | `ros2 launch chassis_bringup rover.launch.py mode:=real camera:=both` |
| no ODESCs (mock) | none | `ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=mock` |
| no ODESCs (mock) | point cloud | `ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=mock camera:=cloud` |
| no ODESCs (mock) | OpenPose | `ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=mock camera:=openpose` |
| no ODESCs (mock) | point cloud + OpenPose | `ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=mock camera:=both` |

With `can_interface:=mock` the driver opens no CAN socket and loops commands back as wheel
feedback; skip `can-up`. Check the drivetrain in a second Jetson terminal:
`ros2 control list_controllers` shows both controllers `active`.

**Ground-station container**, the line matching the Jetson's `camera:=` (same with or without
the ODESCs):

| Jetson `camera:=` | Command |
|---|---|
| none (default) | `ros2 launch chassis_bringup ground_station.launch.py use_sim_time:=false` |
| `cloud` | `ros2 launch chassis_bringup ground_station.launch.py use_sim_time:=false view:=cloud` |
| `openpose` | `ros2 launch chassis_bringup ground_station.launch.py use_sim_time:=false view:=openpose` |
| `both` | `ros2 launch chassis_bringup ground_station.launch.py use_sim_time:=false view:=both` |

`view:=` picks the RViz layout. For Foxglove, import the matching layout ([Foxglove](#foxglove)).
RViz's fixed frame follows `use_sim_time`: `odom` in the sim (the rover moves across the grid),
`base_link` on the real drivetrain, which has no `odom` frame yet (the view moves with the rover).

### Camera rates and RAM (Orin Nano, 2026-11-13)

| Jetson `camera:=` | Real drivetrain (mock) | Sim |
|---|---|---|
| `cloud` | `/zed/points` 8.5 Hz | `/zed/points` 2.8 Hz (10 per second of sim time) |
| `openpose` | `/people/skeletons` 5.0 Hz | `/people/skeletons` 3.9 Hz |
| `both` | both topics 5.0 Hz | `/zed/points` 2.7 Hz, `/people/skeletons` 4.4 Hz |

- The first skeleton arrives about 10 s after launch.
- `openpose` and `both` need about 3000 MB `available` in `free -m` on the Jetson host before
  launch. See [Troubleshooting](#troubleshooting) if OpenPose stops with `out of memory`.
- The rows with the ODESCs on `can2` have not been run yet (no CAN on the dev Jetson). The camera
  part of those commands is the same as with `can_interface:=mock`.

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

   | Jetson runs | Layout |
   |---|---|
   | no `camera:=` | `ros2_ws/src/chassis_bringup/foxglove/drivetrain.json` (3D view, `/joy`, `/diff_drive_controller/cmd_vel_unstamped`) |
   | `camera:=cloud` | `ros2_ws/src/perception/people/foxglove/zed_points.json` |
   | `camera:=openpose` or `camera:=both` | `ros2_ws/src/perception/people/foxglove/people_rover.json` |

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
| | `camera` | `none` | `cloud` (ZED point cloud), `openpose` (OpenPose) or `both`. Jetson container only. |
| `ground_station.launch.py` | `use_sim_time` | `true` | `false` when the Jetson runs `mode:=real` |
| | `view` | `drivetrain` | RViz layout: `drivetrain`, `cloud`, `openpose` or `both` (match the Jetson's `camera:=`) |
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
| RViz/Foxglove: no `odom` frame | Real drivetrain: expected (no EKF yet); `ground_station.launch.py use_sim_time:=false` uses fixed frame `base_link`. Sim: don't pass `navigation:=false`. |
| Sim: `Switch controller timed out`, or `Couldn't open X display` | No usable display. On the Jetson, prefix the launch with `xvfb-run -a vglrun -d egl`. |
| Real: controllers `inactive`, wheels dead | CAN down or wrong bitrate. Run `tooling/can-up` on the host and check `dmesg`. |
| `vcan0` but no frames | Create `vcan0` on the host first, then check with `candump -L vcan0`. |
| `rover.launch.py: camera:=… ignored, zed_openpose_node isn't built here` | You're not in the Jetson container, or `zed_openpose` wasn't built there: `pixi run build --packages-select zed_openpose people`. The drivetrain still runs. |
| `ZED open failed: CAMERA NOT DETECTED` | On the host, `lsusb \| grep 2b03` must list `ZED 2i`. Use a USB 3 port. |
| `[ZED][MCU] Permissions denied`, then `NOT VALID SERIAL NUMBER FOR SENSORS MODULE MCU` | The host has no ZED udev rule. The container installs it at start: on the host, `ls /etc/udev/rules.d/99-slabs.rules`. If it's missing, the container was created before this was added: **Rebuild Container**. |
| OpenPose stops with `out of memory`, `CUDNN_STATUS_INTERNAL_ERROR` or `NvMapMemAlloc… error 12` | Not enough RAM (the GPU shares it). On the Jetson host, `free -m` must show at least 3000 MB `available`. Find what uses it: `ps -eo rss,args --sort=-rss \| head`. On the dev Jetson, the F' GDS on port 5001 grows by about 2 GB an hour; `sudo systemctl restart billee-scm-lan-gds` frees it. VS Code attached to the container can take 1.4 GB. The launch frees the kernel's file cache itself before starting OpenPose. An image built before 2026-11-13 needs **Rebuild Container**. |
| `Could not enable FIFO RT scheduling policy` | Harmless. |
