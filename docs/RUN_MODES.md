# BILLEE Run Modes

Every way to run the ROS 2 stack, written as the plain `ros2 launch` / `ros2 run`
commands. Pick one **chassis** mode (what moves the wheels), one or more **teleop**
modes (who commands it), and a **viewer**; the [recipes](#recipes) combine them
terminal by terminal. The `make` targets are only shortcuts for some of these commands
(see [Shortcuts](#shortcuts-make-and-tooling)).

- [Open a ROS 2 terminal](#open-a-ros-2-terminal) (per platform, no `make`)
- [The contract every mode shares](#the-contract-every-mode-shares)
- [Chassis modes](#chassis-modes--what-drives-the-wheels): C1 sim · C2 sim + ODESC shadow · C3 real · C4 mock · C5 vCAN bench
- [Teleop modes](#teleop-modes--who-commands-the-rover): T1 gamepad · T2 ground station · T3 keyboard · T4 Foxglove · T5 scripted
- [Viewers](#viewers): V1 Foxglove · V2 RViz · V3 F' GDS
- [Recipes](#recipes) · [Platform support](#which-modes-each-platform-supports) · [Launch argument reference](#launch-argument-reference) · [Health check](#quick-health-check-any-mode)

---

## Open a ROS 2 terminal

Every command in this document runs in a **ROS 2 terminal**: the workspace's Pixi
environment activated from `ros2_ws/`. Activation puts ROS 2 Humble on the path, sources
`install/setup.sh` (the built workspace) and sets `ROS_DOMAIN_ID=42`. Each extra
terminal you open (e.g. for keyboard teleop) needs the same step.

| Platform | Pixi env | Get a ROS 2 terminal |
|---|---|---|
| ARM64 Linux (native) | `linux-aarch64` | `cd ~/billee-software-2027/ros2_ws && pixi shell -e linux-aarch64` |
| Mac | `mac-cpu` | VS Code terminal in the **desktop-roshumble-mac-cpu** devcontainer (opens in `/root/ros2_ws`), then `pixi shell -e mac-cpu` |
| x86 + NVIDIA | `default` | VS Code terminal in the **desktop-roshumble_dev-x862** devcontainer, then `cd ros2_ws && pixi shell` |
| Jetson | `l4t` | VS Code terminal in the **rover-roshumble_l4t-aarch64** devcontainer, then `cd ros2_ws && pixi shell -e l4t` |

Without VS Code, the containers are started with `tooling/desktop-ros2 shell` from the
repo root on the host (Jetson: `tooling/rover-ros2 shell`). It opens in the container's
`ros2_ws`; run `pixi shell -e <env>` there. Running it again while that container is up
opens another terminal in the same container.

**One-off commands** don't need a shell: from `ros2_ws`, prefix them with
`pixi run -e <env>`, e.g. `pixi run -e linux-aarch64 ros2 topic list`.

**Build** (first time, and after changing code), from `ros2_ws` in that environment:

```bash
pixi install -e <env>          # first time only: download the environment
pixi run -e <env> build        # colcon build of ros2_ws/src
```

On the Jetson use this for the first build (fits in its RAM), then `pixi run -e l4t build`:

```bash
pixi run -e l4t colcon build --symlink-install --parallel-workers 2 --executor sequential \
  --event-handlers console_direct+ --base-paths src --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Host-side commands.** `tooling/can-up` (creates `can0`/`vcan0`) and `candump`/`cansend`
act on the host kernel's network interfaces: run them in a normal terminal on the host
(Linux or Jetson), from the repo root, not inside a container. The Linux containers use
host networking, so they see the interfaces the host created.

**Headless machines** (Mac container, Jetson over SSH, any machine without a display):
Gazebo needs an X display, so put `xvfb-run -a` in front of any launch that starts the
sim. `xvfb-run` is installed in the Mac and Jetson images and by `make setup linux-aarch64`.

---

## The contract every mode shares

Whatever the mode, the drivetrain looks the same from the outside, so teleop, viewers
and (later) Nav2 don't care which one is running:

| Interface | Type | Direction |
|---|---|---|
| `/diff_drive_controller/cmd_vel_unstamped` | `geometry_msgs/Twist` | **in** — every teleop mode publishes here |
| `/diff_drive_controller/odom` | `nav_msgs/Odometry` | out |
| `/joint_states` | `sensor_msgs/JointState` | out (6 wheel joints) |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | out (`odom → base_link → …`) |
| `/robot_description` | `std_msgs/String` | out (URDF) |
| `ws://<host>:8765` | Foxglove WebSocket | out/in, when the bridge is on |

`ROS_DOMAIN_ID` is pinned to `42` by Pixi, so any two machines on the same LAN that
both go through `pixi` share one ROS graph.

---

## Chassis modes — what drives the wheels

| # | Mode | Launch | Wheels driven by | Needs | Time |
|---|---|---|---|---|---|
| C1 | **Simulation** | `rover.launch.py` (or `sim_gz.launch.py`) | Gazebo physics (`ign_ros2_control`) | display or `xvfb-run` | sim (`/clock`) |
| C2 | **Simulation + ODESC shadow** | `sim_gz.launch.py odesc_shadow:=true` | Gazebo; the real ODESC driver runs alongside on `vcan0` | C1 + `vcan0` on the host | sim |
| C3 | **Real drivetrain** | `rover.launch.py mode:=real` | 6× ODESC V4.2 + NEO over `can0` | the rover, `can0` up | wall |
| C4 | **Mock drivetrain** | `rover.launch.py mode:=real can_interface:=mock` | software loopback in the ODESC driver (no CAN) | nothing | wall |
| C5 | **Virtual CAN bench** | `rover.launch.py mode:=real can_interface:=vcan0` (+ emulator) | ODESC driver ↔ `vcan0` ↔ emulated ODESCs | `vcan0` on the host | wall |

`rover.launch.py` is the rover's entry point: it includes `sim_gz.launch.py` (sim) or
`real.launch.py` (real), always starts the Foxglove bridge on :8765, and takes
`rviz:=true` to also open RViz on the same machine. Use the underlying launch files
directly when you need their extra arguments (spawn pose, ODESC shadow, no bridge).

### C1 — Simulation

The rover in an empty Gazebo Fortress world. Same controllers, topics and TF as the
real rover, so it is the default for developing anything above the drivetrain. It also
starts gamepad teleop on the same machine ([T1](#t1--gamepad-on-the-same-machine)).

```bash
# Linux with a display: Gazebo + RViz window + Foxglove bridge
ros2 launch chassis_bringup rover.launch.py rviz:=true

# Headless (Mac container, Jetson over SSH): Gazebo in xvfb + Foxglove bridge only
xvfb-run -a ros2 launch chassis_bringup rover.launch.py

# The simulator launch on its own, with its extra options
ros2 launch chassis_bringup sim_gz.launch.py rviz:=true foxglove:=true \
  x:=0.0 y:=0.0 z:=0.2 yaw:=0.0
```

Wait for `Configured and activated diff_drive_controller` in the log, then drive with
any teleop mode. Stop everything with Ctrl-C in that terminal.

### C2 — Simulation + ODESC shadow

Gazebo stays the physics and TF source, while a second, `/odesc_shadow`-namespaced
controller manager runs the **real** `OdescSystemHardware` plugin against `vcan0`, fed
the same `cmd_vel`. An emulator answers as the six ODESCs. Use it to check the CAN
frames your commands turn into, without motors. Linux hosts only (not the Mac).

```bash
# Host terminal (repo root), once per boot:
tooling/can-up vcan0

# ROS 2 terminal:
ros2 launch chassis_bringup sim_gz.launch.py odesc_shadow:=true foxglove:=true rviz:=true
#   headless: xvfb-run -a ros2 launch chassis_bringup sim_gz.launch.py odesc_shadow:=true foxglove:=true

# Host terminal: watch the frames (0x07 on activate, 0x0D per command)
candump -L vcan0

# ROS 2 terminal: shadow odometry, to compare with Gazebo's /diff_drive_controller/odom
ros2 topic echo /odesc_shadow/diff_drive_controller/odom
```

Tear the bus down afterwards with `tooling/can-up down vcan0` (host).

### C3 — Real drivetrain

The actual rover: `OdescSystemHardware` talks ODrive CANSimple to the six ODESCs on
`can0` at 500 kbit/s, applying the 48:1 gear ratio. The CAN node ↔ wheel map is in the
URDF (canonical copy: `odesc/config/node_map.yaml`); nothing to pass.

```bash
# Jetson host terminal (repo root): bring can0 up at 500 kbit/s
tooling/can-up
tooling/can-up status can0

# ROS 2 terminal on the Jetson: drivetrain + Foxglove bridge
ros2 launch chassis_bringup rover.launch.py mode:=real
#   other ratio: ... gear_ratio:=<n>
#   without the bridge: ros2 launch chassis_bringup real.launch.py

# ROS 2 terminal: check it came up
ros2 control list_hardware_components     # 'Robot' ACTIVE
ros2 control list_controllers             # both active
```

The real launch does **not** start teleop: drive from a ground station
([T2](#t2--gamepad-at-a-ground-station)), or plug the pad into the Jetson and run
[T1](#t1--gamepad-on-the-same-machine). Viewers run on the ground station
(`use_sim_time:=false`).

> **Safety:** first runs with the rover on blocks. Motion only happens while the
> gamepad deadman is held; releasing it sends zero, and `diff_drive_controller`
> stops the wheels 0.25 s after the last command (`cmd_vel_timeout`).

Persist `can0` across reboots with `tooling/can0.service` (see
[RUN_GUIDE](RUN_GUIDE.md#b-real-drivetrain-odesc-over-can)).

### C4 — Mock drivetrain

The full real (non-Gazebo) stack — standalone `controller_manager`, ODESC plugin,
odometry, TF, viewers — with the plugin looping commanded velocity back as encoder
feedback. No CAN, no motors, no Gazebo, no GPU, so it runs anywhere (including the Mac
container) and is the fastest way to test teleop and viewers.

```bash
ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=mock
#   + RViz on this machine:  ... rviz:=true
```

Like C3 it starts no teleop; add [T1](#t1--gamepad-on-the-same-machine),
[T3](#t3--keyboard) or [T5](#t5--scripted) in another terminal, and use
`use_sim_time:=false` for any separate viewer.

### C5 — Virtual CAN bench

The real launch on a virtual bus. Without the emulator you inject encoder frames by
hand with `cansend`; with it, the whole CAN loop runs with no hardware and no Gazebo.
Linux hosts only.

```bash
# Host terminal (repo root):
tooling/can-up vcan0

# ROS 2 terminal 1: drivetrain on vcan0 + Foxglove bridge
ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=vcan0

# ROS 2 terminal 2 (optional): six emulated ODESCs answering on vcan0
ros2 run odesc odesc_vcan_emulator.py --interface vcan0

# Host terminal: watch the traffic, then tear down when finished
candump -L vcan0
tooling/can-up down vcan0
```

---

## Teleop modes — who commands the rover

| # | Mode | How | Runs on | Needs |
|---|---|---|---|---|
| T1 | **Gamepad, same machine** | started by the sim (C1/C2), else `teleop.launch.py` | wherever the pad is plugged in | `/dev/input/js0` |
| T2 | **Gamepad at a ground station** | `ground_station.launch.py` | ground station, over DDS to the rover | same LAN + `ROS_DOMAIN_ID` |
| T3 | **Keyboard** | `teleop_twist_keyboard` | any ROS 2 terminal on the graph | nothing |
| T4 | **Foxglove Teleop panel** | Foxglove Studio → Teleop panel | any laptop with Studio | the bridge (:8765) |
| T5 | **Scripted** | `ros2 topic pub` | any ROS 2 terminal on the graph | nothing |

### Gamepad controls (T1, T2)

Arcade drive (`teleop/src/joy_teleop.cpp`, tuned in `teleop/config/joystick.yaml`; the
sim's copy is `chassis_bringup/config/tele_params.yaml`):

| Input | Action |
|---|---|
| **Hold RB / R1** (button 5) | deadman — nothing moves unless held; release = stop |
| **Right trigger RT / R2** (axis 5) | forward, proportional to how far it's pulled |
| **Left trigger LT / L2** (axis 2) | reverse |
| **Left stick, horizontal** (axis 0) | steer left / right |

`linear.x = (RT − LT) · speed_scale` (2.0 m/s at full trigger), `angular.z = stick · steer_scale`
(1.5 rad/s at full stick); all zero while RB is released. Triggers read as released until
they are first pulled (some drivers report 0 before that). Every index, scale and the
trigger rest/pressed values are parameters — check with `ros2 topic echo /joy` when
using a different pad, and set `invert_steer: true` if steering is mirrored.

### T1 — Gamepad on the same machine

The simulation launches (C1/C2) start the gamepad nodes automatically. For the real,
mock and vCAN modes, start them in another ROS 2 terminal:

```bash
ros2 launch teleop teleop.launch.py      # joy_node + joy_tank_drive -> /diff_drive_controller/cmd_vel_unstamped
```

Check the pad before driving:

```bash
ls /dev/input/js*                        # host: the pad must show up (js0)
ros2 topic echo /joy                     # buttons[5] = 1 with RB held; axes[0/2/5] move with stick/LT/RT
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped   # non-zero only with RB held
```

Works on Linux (x86 container, ARM64 Linux, Jetson). Not in the Mac container (Docker
Desktop cannot pass a USB gamepad through) — use T3 or T4 there.

### T2 — Gamepad at a ground station

The operator's machine runs teleop + RViz + a local Foxglove bridge; commands cross to
the rover's `diff_drive_controller` over DDS. The rover runs C1 or C3/C4.

```bash
# Ground-station ROS 2 terminal:
ros2 launch chassis_bringup ground_station.launch.py                      # rover in sim
ros2 launch chassis_bringup ground_station.launch.py use_sim_time:=false  # rover on C3/C4
#   options: rviz:=false  joystick:=false  foxglove:=false  fprime_gds:=true
```

Check the link first, from the ground station:

```bash
ros2 topic list                          # must show /diff_drive_controller/*, /tf, /robot_description
```

Details: [RUN_GUIDE → Cross-machine ROS 2](RUN_GUIDE.md#cross-machine-ros-2). Needs a
real LAN path between the machines: not from the Mac container, and a Linux VM needs
bridged networking.

### T3 — Keyboard

In its own ROS 2 terminal (it reads keys from that terminal):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

`i` forward, `,` back, `j`/`l` turn, `k` stop; `q`/`z` raise/lower speed.

### T4 — Foxglove Teleop panel

In Foxglove Studio connected to `ws://<host>:8765` ([V1](#v1--foxglove-studio)): add a
**Teleop** panel and set its topic to `/diff_drive_controller/cmd_vel_unstamped`. One
WebSocket, no DDS — the way to drive from a Mac or over a link where DDS does not work.

### T5 — Scripted

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist '{linear: {x: 0.4}, angular: {z: 0.3}}'   # drives a circle
```

The quickest end-to-end test of any chassis mode or cross-machine link. Stop with
Ctrl-C; the controller times out to zero after 0.25 s.

---

## Viewers

Both V1 and V2 show the same layout: grid, TF, robot model, odometry trail, fixed frame
`odom`. Use `use_sim_time:=true` when viewing the simulation and `false` for C3/C4/C5.

### V1 — Foxglove Studio

The bridge is already running with `rover.launch.py` and `ground_station.launch.py`,
and with `foxglove:=true` on `sim_gz.launch.py` / `real.launch.py`. To add just the
bridge to a running system:

```bash
ros2 launch chassis_bringup viz.launch.py rviz:=false use_sim_time:=true   # false for real/mock
```

Then in Foxglove Studio: **Open connection → Foxglove WebSocket →** `ws://<host>:8765`
(`ws://localhost:8765` on the same machine or from the Mac host) → **Layouts → Import
from file →** `ros2_ws/src/chassis_bringup/foxglove/drivetrain.json`. The one to use
from a Mac or over the radio link.

### V2 — RViz2

`rviz:=true` on `rover.launch.py`, `sim_gz.launch.py` or `real.launch.py`, or on its own
next to a running system:

```bash
ros2 launch chassis_bringup viz.launch.py foxglove:=false use_sim_time:=true   # false for real/mock
```

Opens a window with `rviz/drivetrain.rviz`, so it needs a display (not the Mac
container). From a ground station it shows the rover over DDS.

### V3 — F' GDS

```bash
ros2 launch chassis_bringup ground_station.launch.py fprime_gds:=true
#   other address for one run: ... fprime_gds_url:=http://<host>:<port>
```

Opens the F' GDS web UI in the default browser at `$FPRIME_GDS_URL` (set in
`ros2_ws/pixi.toml`). Needs a browser and a graphical session on that machine.

---

## Recipes

Each step is one terminal. "ROS 2 terminal" means [opened as above](#open-a-ros-2-terminal)
for that machine.

### Develop on a Linux machine (sim + gamepad + RViz)

1. ROS 2 terminal: `ros2 launch chassis_bringup rover.launch.py rviz:=true`
2. Hold RB, pull RT to drive, steer with the left stick. No pad? Second ROS 2 terminal
   with [T3](#t3--keyboard).

### Develop on a Mac (headless sim + Foxglove)

1. Mac devcontainer terminal: `pixi shell -e mac-cpu`, then
   `xvfb-run -a ros2 launch chassis_bringup rover.launch.py`
2. Foxglove Studio on the Mac: `ws://localhost:8765`, import the layout ([V1](#v1--foxglove-studio)).
3. Drive with a Foxglove Teleop panel ([T4](#t4--foxglove-teleop-panel)), or a second
   container terminal with [T3](#t3--keyboard).

For the fastest loop without Gazebo, step 1 can be the mock:
`ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=mock`.

### Test CAN framing against the sim

1. Host: `tooling/can-up vcan0`
2. ROS 2 terminal: `ros2 launch chassis_bringup sim_gz.launch.py odesc_shadow:=true foxglove:=true rviz:=true`
3. Host: `candump -L vcan0`
4. Drive (gamepad or [T5](#t5--scripted)) and watch the `0x0D` frames follow.

### Bench-test the real stack without hardware

1. ROS 2 terminal: `ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=mock rviz:=true`
2. ROS 2 terminal: `ros2 launch teleop teleop.launch.py` (or [T5](#t5--scripted))

### Rover on blocks / field operations

1. Jetson host: `tooling/can-up`
2. Jetson ROS 2 terminal: `ros2 launch chassis_bringup rover.launch.py mode:=real`
3. Ground station ROS 2 terminal: `ros2 launch chassis_bringup ground_station.launch.py use_sim_time:=false`
   (over a radio link, add `rviz:=false` and use Foxglove at `ws://<rover-ip>:8765`)
4. Hold RB on the ground-station pad and drive. Keep a Foxglove Teleop panel as backup.

### Simulated rover with a remote operator

1. Jetson ROS 2 terminal: `xvfb-run -a ros2 launch chassis_bringup rover.launch.py`
2. Ground station ROS 2 terminal: `ros2 launch chassis_bringup ground_station.launch.py`

(The Jetson sim also starts its own joystick nodes; with no pad plugged into the
Jetson they stay idle and the ground station's pad drives.)

---

## Shortcuts (`make` and `tooling/`)

From the repo root, after `make setup <platform>`; each picks the platform's Pixi env
and, where needed, runs inside its container:

| Shortcut | Plain equivalent |
|---|---|
| `make shell` | open a ROS 2 terminal as in [the table above](#open-a-ros-2-terminal) |
| `make build` | `pixi run -e <env> build` |
| `make sim` / `tooling/sim-up` | Linux with a display: `ros2 launch chassis_bringup sim_gz.launch.py foxglove:=true rviz:=true`; Mac / headless: `xvfb-run -a ros2 launch chassis_bringup sim_gz.launch.py foxglove:=true`. Adds `odesc_shadow:=true` when `vcan0` exists. |

---

## Which modes each platform supports

| | x86 + NVIDIA | ARM64 Linux (native) | Mac container | Jetson (`l4t`) |
|---|---|---|---|---|
| C1 Simulation | ✅ GPU | ✅ CPU (slower) | ✅ headless, `xvfb-run` | ✅ headless, `xvfb-run` |
| C2 Sim + shadow / C5 vCAN | ✅ | ✅ | ❌ no host `vcan0` | ✅ |
| C3 Real drivetrain | with a USB-CAN adapter | with a USB-CAN adapter | ❌ | ✅ `can0` |
| C4 Mock | ✅ | ✅ | ✅ | ✅ |
| T1 Gamepad | ✅ | ✅ | ❌ | ✅ |
| T2 Ground station (DDS) | ✅ | ✅ (VM: bridged net) | ❌ | — (it's the rover) |
| T3 / T4 / T5 | ✅ | ✅ | ✅ | ✅ |
| V2 RViz window | ✅ | ✅ | ❌ | with a display |

---

## Launch argument reference

All in `chassis_bringup/launch/` except `teleop.launch.py`. Pass as `name:=value`.

**`rover.launch.py`** — rover entry point; always starts the Foxglove bridge.

| Argument | Default | Meaning |
|---|---|---|
| `mode` | `sim` | `sim` → `sim_gz.launch.py`, `real` → `real.launch.py` |
| `rviz` | `false` | also open RViz here |
| `can_interface` | `can0` | `mode:=real` only: `can0`, `vcan0`, or `mock`/`none` |
| `gear_ratio` | `48.0` | `mode:=real` only: motor turns per wheel turn |

**`sim_gz.launch.py`** — Gazebo backend, bridges, spawn, controllers, joystick teleop.

| Argument | Default | Meaning |
|---|---|---|
| `x`, `y`, `z`, `yaw` | `0.0`, `0.0`, `0.2`, `0.0` | spawn pose (m, rad) |
| `rviz` | `false` | also open RViz |
| `foxglove` | `false` | also start the Foxglove bridge |
| `odesc_shadow` | `false` | also run the ODESC driver + emulator on `vcan0` |
| `joy_control` | `true` | joystick nodes (currently always started — known bug) |
| `description_pkg`, `xacro_file` | `robot_description`, `urdf/robot.urdf.xacro` | robot model source |
| `world_file` | `empty.sdf` | accepted but ignored (always `empty.sdf`) |

**`real.launch.py`** — hardware backend with a standalone controller manager (wall clock).

| Argument | Default | Meaning |
|---|---|---|
| `can_interface` | `can0` | `can0`, `vcan0`, or `mock`/`none` (no CAN) |
| `gear_ratio` | `48.0` | motor turns per wheel turn |
| `rviz` | `false` | also open RViz |
| `foxglove` | `false` | also start the Foxglove bridge |
| `description_pkg`, `xacro_file`, `controllers_file` | `robot_description`, `urdf/robot.urdf.xacro`, `config/controllers.yaml` | model and controller config |

**`ground_station.launch.py`** — operator station: viewers + teleop.

| Argument | Default | Meaning |
|---|---|---|
| `use_sim_time` | `true` | `false` when the rover is on real/mock hardware |
| `rviz` | `true` | open RViz |
| `foxglove` | `true` | local Foxglove bridge on :8765 |
| `joystick` | `true` | gamepad teleop (`teleop.launch.py`) |
| `fprime_gds` | `false` | open the F' GDS web UI |
| `fprime_gds_url` | `$FPRIME_GDS_URL` | address to open |

**`viz.launch.py`** — viewers only.

| Argument | Default | Meaning |
|---|---|---|
| `rviz` | `true` | open RViz |
| `foxglove` | `true` | Foxglove bridge on :8765 |
| `use_sim_time` | `false` | `true` when viewing the sim |
| `rviz_config` | `rviz/drivetrain.rviz` | RViz config file |

**`odesc_shadow.launch.py`** — normally included by `sim_gz.launch.py odesc_shadow:=true`;
`can_interface` (default `vcan0`), `description_pkg`, `xacro_file`.

**`teleop/launch/teleop.launch.py`** — no arguments; parameters in `teleop/config/joystick.yaml`.

---

## Quick health check (any mode)

```bash
ros2 control list_controllers                          # diff_drive_controller + joint_state_broadcaster active
ros2 topic hz /diff_drive_controller/odom              # ~50 Hz
ros2 topic echo --once /joint_states                   # six wheel joints
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped   # your teleop is arriving
```

More symptoms and fixes: [RUN_GUIDE → Troubleshooting](RUN_GUIDE.md#troubleshooting).
