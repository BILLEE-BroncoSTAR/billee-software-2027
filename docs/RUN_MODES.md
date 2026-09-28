# BILLEE Run Modes

Every way to run the ROS 2 stack, grouped by what you are exercising. Pick one
**chassis** mode (what moves the wheels), one or more **teleop** modes (who commands
it), and a **viewer**. The [recipes](#putting-it-together-recipes) at the end combine
them for common jobs.

All commands assume a ROS 2 shell for your machine (after `make setup <platform>`, see
the top-level [README](../README.md#setup)):

```bash
make shell                  # from the repo root; then run the commands below as-is
# or, one-off:  cd ros2_ws && pixi run -e <env> <command>
```

`<env>` is `default` (x86), `mac-cpu` (Mac container / ARM64 Linux) or `l4t` (Jetson).
Gazebo needs a display: on a headless machine put `xvfb-run -a` in front of any launch
that starts the sim.

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
| C1 | **Simulation** | `rover.launch.py` | Gazebo physics (`ign_ros2_control`) | display or `xvfb-run` | sim (`/clock`) |
| C2 | **Simulation + ODESC shadow** | `sim_gz.launch.py odesc_shadow:=true`, or `tooling/sim-up` / `make sim` once `vcan0` exists | Gazebo; the real ODESC driver runs alongside on `vcan0` | C1 + `vcan0` on the host | sim |
| C3 | **Real drivetrain** | `rover.launch.py mode:=real` | 6× ODESC V4.2 + NEO over `can0` | the rover, `can0` up | wall |
| C4 | **Mock drivetrain** | `rover.launch.py mode:=real can_interface:=mock` | software loopback in the ODESC driver (no CAN) | nothing | wall |
| C5 | **Virtual CAN bench** | `real.launch.py can_interface:=vcan0` (+ emulator) | ODESC driver ↔ `vcan0` ↔ emulated ODESCs | `vcan0` on the host | wall |

`rover.launch.py` always starts the Foxglove bridge (:8765) and takes `rviz:=true` to
open RViz on the same machine.

### C1 — Simulation

The rover in an empty Gazebo Fortress world. Same controllers, topics and TF as the
real rover, so it is the default for developing anything above the drivetrain.

```bash
make sim                                               # from the repo root: picks the viewer for you
ros2 launch chassis_bringup rover.launch.py            # + foxglove bridge
ros2 launch chassis_bringup rover.launch.py rviz:=true # + RViz on this machine
xvfb-run -a ros2 launch chassis_bringup rover.launch.py   # headless (Mac container, Jetson over SSH)
```

Also starts gamepad teleop (`joy_node` + `joy_tank_drive`) on the same machine — see
[T1](#t1--gamepad-on-the-same-machine). Spawn pose: `sim_gz.launch.py x:= y:= z:= yaw:=`.

### C2 — Simulation + ODESC shadow

Gazebo stays the physics and TF source, while a second, `/odesc_shadow`-namespaced
controller manager runs the **real** `OdescSystemHardware` plugin against `vcan0`, fed
the same `cmd_vel`. An emulator answers as the six ODESCs. Use it to check the CAN
frames your commands turn into, without motors.

```bash
tooling/can-up vcan0                      # once, on the host (not inside a container)
tooling/sim-up                            # sees vcan0 -> odesc_shadow:=true, + gamepad + foxglove
candump -L vcan0                          # watch 0x07 on activate, 0x0D per command
ros2 topic echo /odesc_shadow/diff_drive_controller/odom   # shadow odometry vs Gazebo's
```

### C3 — Real drivetrain

The actual rover: `OdescSystemHardware` talks ODrive CANSimple to the six ODESCs on
`can0` at 500 kbit/s, applying the 48:1 gear ratio. The CAN node ↔ wheel map is in the
URDF (canonical copy: `odesc/config/node_map.yaml`); nothing to pass.

```bash
tooling/can-up                            # can0 @ 500 kbit/s (Jetson host)
ros2 launch chassis_bringup rover.launch.py mode:=real
ros2 control list_hardware_components     # 'Robot' ACTIVE
ros2 control list_controllers             # both active
```

> **Safety:** first runs with the rover on blocks. Motion only happens while the
> gamepad deadman is held; releasing it sends zero, and `diff_drive_controller`
> stops the wheels 0.25 s after the last command (`cmd_vel_timeout`).

Options: `gear_ratio:=<n>`; persist `can0` across reboots with `tooling/can0.service`
(see [RUN_GUIDE](RUN_GUIDE.md#b-real-drivetrain-odesc-over-can)).

### C4 — Mock drivetrain

The full real (non-Gazebo) stack — standalone `controller_manager`, ODESC plugin,
odometry, TF, viewers — with the plugin looping commanded velocity back as encoder
feedback. No CAN, no motors, no Gazebo, no GPU, so it runs anywhere (including the Mac
container) and is the fastest way to test teleop and viewers.

```bash
ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=mock
```

### C5 — Virtual CAN bench

The real launch on a virtual bus. Without the emulator you inject encoder frames by
hand; with it, the whole CAN loop runs with no hardware and no Gazebo.

```bash
tooling/can-up vcan0                                       # host
ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=vcan0
ros2 run odesc odesc_vcan_emulator.py --interface vcan0    # optional: 6 emulated ODESCs
candump -L vcan0
tooling/can-up down vcan0                                  # tear down afterwards
```

---

## Teleop modes — who commands the rover

| # | Mode | How | Runs on | Needs |
|---|---|---|---|---|
| T1 | **Gamepad, same machine** | started by the sim (C1/C2), or `ros2 launch teleop teleop.launch.py` | wherever the pad is plugged in | `/dev/input/js0` |
| T2 | **Gamepad at a ground station** | `ros2 launch chassis_bringup ground_station.launch.py` | ground station, over DDS to the rover | same LAN + `ROS_DOMAIN_ID` |
| T3 | **Keyboard** | `teleop_twist_keyboard` | any terminal on the ROS graph | nothing |
| T4 | **Foxglove Teleop panel** | Foxglove Studio → Teleop panel | any laptop with Studio | the bridge (:8765) |
| T5 | **Scripted** | `ros2 topic pub` | any terminal on the ROS graph | nothing |

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

The simulation launches start the gamepad nodes automatically. For the real/mock modes,
start them yourself:

```bash
ros2 launch teleop teleop.launch.py
```

Works natively on Linux (x86 container, ARM64 Linux, Jetson). Not in the Mac container
(Docker Desktop cannot pass a USB gamepad through) — use T3 or T4 there.

### T2 — Gamepad at a ground station

The operator's laptop runs teleop + RViz + a local Foxglove bridge; commands cross to
the rover's `diff_drive_controller` over DDS.

```bash
ros2 launch chassis_bringup ground_station.launch.py                      # rover in sim
ros2 launch chassis_bringup ground_station.launch.py use_sim_time:=false  # rover on C3/C4
#   rviz:=false  joystick:=false  foxglove:=false  fprime_gds:=true  (open the F' GDS UI)
```

Check the link first: `ros2 topic list` on the ground station must show
`/diff_drive_controller/*`. Details: [RUN_GUIDE → Cross-machine ROS 2](RUN_GUIDE.md#cross-machine-ros-2).
Needs a real LAN path: not from the Mac container, and a Linux VM needs bridged networking.

### T3 — Keyboard

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

Runs in its own terminal (it reads stdin). `i`/`,` forward/back, `j`/`l` turn, `k` stop.

### T4 — Foxglove Teleop panel

Connect Foxglove Studio to `ws://<host>:8765`, add a **Teleop** panel and set its topic
to `/diff_drive_controller/cmd_vel_unstamped`. One WebSocket, no DDS — the way to drive
from a Mac or over a link where DDS does not work.

### T5 — Scripted

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist '{linear: {x: 0.4}, angular: {z: 0.3}}'   # drives a circle
```

The quickest end-to-end test of any chassis mode or cross-machine link. Stop with
Ctrl-C; the controller times out to zero.

---

## Viewers

| # | Viewer | Start | Open |
|---|---|---|---|
| V1 | **Foxglove Studio** | on by default in `rover.launch.py` and `ground_station.launch.py`; else `foxglove:=true` or `viz.launch.py rviz:=false` | Studio → `ws://<host>:8765` → Layouts → Import `ros2_ws/src/chassis_bringup/foxglove/drivetrain.json` |
| V2 | **RViz2** | `rviz:=true` on any bring-up, or `ros2 launch chassis_bringup viz.launch.py foxglove:=false use_sim_time:=<true in sim>` | opens a window (needs a display) |
| V3 | **F' GDS** | `ground_station.launch.py fprime_gds:=true` | browser at `$FPRIME_GDS_URL` (set in `ros2_ws/pixi.toml`) |

Both V1 and V2 show the same layout: grid, TF, robot model, odometry trail, fixed frame
`odom`. Foxglove is the one to use over the radio link or from a Mac.

---

## Putting it together (recipes)

| Job | Chassis | Teleop | Viewer | Commands |
|---|---|---|---|---|
| **Develop on a laptop** | C1 | T1 or T3 | V2 | `ros2 launch chassis_bringup rover.launch.py rviz:=true` |
| **Develop on a Mac** | C1 (headless) or C4 | T3 / T4 | V1 on the Mac host | `make sim` → Studio `ws://localhost:8765` ([walkthrough](../README.md#apple-silicon-mac-mac)) |
| **Develop on Linux** | C1 | T1 | V2 (+V1) | `make sim` (Gazebo + RViz window) |
| **Test CAN framing** | C2 or C5 | T1 / T5 | V1 | `tooling/can-up vcan0` → `tooling/sim-up` → `candump -L vcan0` |
| **Bench-test the real stack** | C4 | T1 / T5 | V1 / V2 | `rover.launch.py mode:=real can_interface:=mock` |
| **Rover on blocks** | C3 | T2 | V2 on ground | rover: `tooling/can-up && rover.launch.py mode:=real` · ground: `ground_station.launch.py use_sim_time:=false` |
| **Field ops** | C3 | T2 (+T4 backup) | V1 over radio | as above; ground can drop RViz (`rviz:=false`) and use Studio |
| **Sim with a remote operator** | C1 on the Jetson | T2 | V2 on ground | rover: `xvfb-run -a rover.launch.py` · ground: `ground_station.launch.py` |

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

## Quick health check (any mode)

```bash
ros2 control list_controllers                          # diff_drive_controller + joint_state_broadcaster active
ros2 topic hz /diff_drive_controller/odom              # ~50 Hz
ros2 topic echo --once /joint_states                   # six wheel joints
ros2 topic echo /diff_drive_controller/cmd_vel_unstamped   # your teleop is arriving
```

More symptoms and fixes: [RUN_GUIDE → Troubleshooting](RUN_GUIDE.md#troubleshooting).
