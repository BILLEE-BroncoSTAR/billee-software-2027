# BILLEE Drivetrain — Run Guide

## Machines

| tag | machine | Pixi env | runs |
|-----|---------|----------|------|
| **[rover]** | Jetson, headless | `l4t` | bring-up (sim/real), `controller_manager` + controllers, `foxglove_bridge` |
| **[ground]** | x86 Linux laptop/desktop, has a monitor | `default` | teleop (gamepad plugged in here), RViz, Foxglove Studio |

Both machines: same repo at `~/billee-software-2027/ros2_ws`, same LAN, same `ROS_DOMAIN_ID`
(pinned to `42` in `ros2_ws/pixi.toml` — see [Cross-machine ROS 2](#cross-machine-ros-2)).

Every terminal, first:

```bash
cd ~/billee-software-2027/ros2_ws
export PATH="$HOME/.pixi/bin:$PATH"
```

Command form: `pixi run --environment <env> <cmd>` — `<env>` is `l4t` on **[rover]**,
`default` on **[ground]**. Commands below already have the right env.

---

## Quick launch (2 commands)

One launch per machine — you run **both** (on one box for local sim, or one each for a
real rover + ground station). Sections A/B below are the manual, per-node breakdown for
debugging.

**[rover]**
```bash
# sim (Gazebo, no ODESC):
xvfb-run -a pixi run --environment l4t ros2 launch chassis_bringup rover.launch.py
# real drivetrain (ODESC over CAN — bring up can0 first, see B.1):
pixi run --environment l4t ros2 launch chassis_bringup rover.launch.py mode:=real
#   mode:=real also takes can_interface:= (can0 | mock | vcan0) and gear_ratio:=
```
`rover.launch.py` runs the right backend **and** the `foxglove_bridge` (:8765). No viewer,
no teleop, no browser — those are all on `ground_station.launch.py` below.

**[ground]**
```bash
pixi run --environment default ros2 launch chassis_bringup ground_station.launch.py fprime_gds:=true
#   drop fprime_gds:=true  to skip opening the GDS web UI
#   use_sim_time:=false    when the rover is on real hardware (not sim)
```
`ground_station.launch.py` runs the joystick (teleop) + RViz + a local `foxglove_bridge`
(:8765) — **this** is the launch with `fprime_gds:=true` (needs a real browser + a
graphical session on that machine; it's a no-op in a headless container). Open Foxglove
Studio → `ws://localhost:8765` → import
`ros2_ws/src/chassis_bringup/foxglove/drivetrain.json`.

### Setting the F' GDS address

`fprime_gds:=true` opens the GDS web UI. The address resolves in this order (first wins):
`fprime_gds_url:=` arg → `$FPRIME_GDS_URL` → the launch file's built-in fallback. The
checked-in value is in `ros2_ws/pixi.toml` (`[activation.env]` → `FPRIME_GDS_URL`). Pick one:

1. **Permanent, whole team** — edit `FPRIME_GDS_URL` under `[activation.env]` in
   `ros2_ws/pixi.toml` and commit. Every `pixi run` / `pixi shell` uses it.
2. **Your shell only** — `export FPRIME_GDS_URL=http://<host>:<port>` (add to `~/.bashrc`
   to keep it), then launch normally.
3. **One run** —
   `... ros2 launch chassis_bringup ground_station.launch.py fprime_gds:=true fprime_gds_url:=http://<host>:<port>`

In every case you still need `fprime_gds:=true` for the browser to open.

---

## Build (once, and after code changes)

**[rover]**
```bash
pixi run --environment l4t build
```

**[ground]**
```bash
pixi run --environment default build
```

Verify (run on the machine you built, matching env):
```bash
pixi run --environment l4t ros2 pkg list | grep -E 'chassis_bringup|odesc|robot_description|teleop'
```
All four packages must print.

First build only, if `pixi run ... build` is not set up:
```bash
pixi run --environment l4t -- colcon build --symlink-install \
  --parallel-workers 2 --executor sequential --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release'
```

---

## A. Simulation (Gazebo, no CAN hardware)

**1. [rover] — sim + robot + controllers**
```bash
xvfb-run -a pixi run --environment l4t ros2 launch chassis_bringup sim_gz.launch.py
```
Drop `xvfb-run -a` only if the Jetson has a monitor and you want the Gazebo window.

**2. [rover] — Foxglove bridge** (port 8765)
```bash
pixi run --environment l4t ros2 launch chassis_bringup viz.launch.py rviz:=false use_sim_time:=true
```

**3. [ground] — RViz**
```bash
pixi run --environment default ros2 launch chassis_bringup viz.launch.py rviz:=true foxglove:=false use_sim_time:=true
```

**4. [ground] — gamepad** (plugged into the ground station)
```bash
pixi run --environment default ros2 launch teleop teleop.launch.py
```
Publishes `/diff_drive_controller/cmd_vel_unstamped`; it reaches the Jetson's
`diff_drive_controller` over DDS (same `ROS_DOMAIN_ID` + LAN — see
[Cross-machine ROS 2](#cross-machine-ros-2)). No `use_sim_time` — teleop runs on wall
clock even though the sim is on `/clock`.

**5. [ground] — Foxglove Studio** (optional, alternative to RViz)
Open connection → `ws://<jetson-ip>:8765` (`hostname -I` on the rover). Then
Layouts → Import from file → `ros2_ws/src/chassis_bringup/foxglove/drivetrain.json`
(same content as `drivetrain.rviz`: grid, TF, robot model, `odom` trail, fixed frame
`odom`).

**Drive:** hold **RB / R1** (deadman); **RT / R2** = forward, **LT / L2** = reverse; **left
stick** = steer (push right → turn right). Release the deadman = stop. Xbox and PlayStation
pads both work.

**Drive without a gamepad — [ground]:** (also the quickest cross-machine link test)
```bash
pixi run --environment default ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist '{linear: {x: 0.4}, angular: {z: 0.3}}'
```
The Gazebo rover on the Jetson should drive a circle.

---

## B. Real drivetrain (ODESC over CAN)

**1. [rover] — bring up the real CAN bus** (`can0`)
```bash
~/billee-software-2027/tooling/can-up            # can0 @ 500000 bit/s
~/billee-software-2027/tooling/can-up status can0
```
Persist across reboots: `sudo cp tooling/can0.service /etc/systemd/system/ && sudo systemctl enable --now can0.service`
(Skip this step for `can_interface:=mock`; for `:=vcan0` run `tooling/can-up vcan0` instead.)

**2. [rover] — drivetrain**
```bash
pixi run --environment l4t ros2 launch chassis_bringup real.launch.py
```
Bare command = real ODESC bus (`can0`) at the 48:1 gear ratio. **The CAN node → wheel
mapping is already in the URDF** (`ros2_control.urdf.xacro`, canonical
`odesc/config/node_map.yaml`) — `real.launch.py` expands it for you, there is nothing to
pass. Everything below is an *optional* arg with a default:

| arg | default | meaning |
|---|---|---|
| `can_interface:=` | `can0` | `can0` real bus · `vcan0` virtual bus · `mock` no CAN — see [CAN backend](#can-backend-can_interface) |
| `gear_ratio:=` | `48.0` | motor-shaft turns per wheel turn |
| `foxglove:=` | `false` | `true` folds the bridge in (skip step 3) |

`rover.launch.py mode:=real` is the one-command form — it forwards `can_interface:=` /
`gear_ratio:=` and turns the bridge on.

**No ODESC hardware yet?** Pick one:
```bash
# full control stack, no CAN at all — /odom, TF and the viewers still work:
pixi run --environment l4t ros2 launch chassis_bringup real.launch.py can_interface:=mock

# CAN-protocol test on a virtual bus (inject / watch frames by hand):
~/billee-software-2027/tooling/can-up vcan0
pixi run --environment l4t ros2 launch chassis_bringup real.launch.py can_interface:=vcan0
```

**3. [rover] — Foxglove bridge**
```bash
pixi run --environment l4t ros2 launch chassis_bringup viz.launch.py rviz:=false use_sim_time:=false
```

**4. [ground] — RViz**
```bash
pixi run --environment default ros2 launch chassis_bringup viz.launch.py rviz:=true foxglove:=false use_sim_time:=false
```
Or, instead of RViz, Foxglove Studio → connect `ws://<jetson-ip>:8765` →
Layouts → Import from file → `ros2_ws/src/chassis_bringup/foxglove/drivetrain.json`.

**5. [ground] — gamepad** (plugged into the ground station)
```bash
pixi run --environment default ros2 launch teleop teleop.launch.py
```
Reaches the Jetson's `diff_drive_controller` over DDS — see
[Cross-machine ROS 2](#cross-machine-ros-2).

**6. [rover] — verify hardware**
```bash
pixi run --environment l4t ros2 control list_hardware_components   # 'Robot' = ACTIVE
pixi run --environment l4t ros2 control list_controllers           # both 'active'
```

**Drive:** same as sim (hold RB/R1; RT/R2 forward, LT/L2 reverse; left stick steers).

---

## Reference

### RViz vs Foxglove — how to choose

Both read the same topics (`/robot_description`, `/tf`, `/diff_drive_controller/odom`), show
the same thing, and can run at the same time. `viz.launch.py` has two independent booleans
`rviz:=` and `foxglove:=` — that is the switch. Pick per session:

- **RViz** — runs on **[ground]**. Needs a local display and the `default` workspace built
  there (meshes + `.rviz` config resolve from the local checkout). Launch:
  `pixi run --environment default ros2 launch chassis_bringup viz.launch.py rviz:=true foxglove:=false use_sim_time:=<t>`
- **Foxglove** — **[rover]** runs only the bridge
  (`... viz.launch.py rviz:=false foxglove:=true use_sim_time:=<t>`, or `foxglove:=true`
  folded into `sim_gz.launch.py` / `real.launch.py`). **[ground]** opens Foxglove Studio,
  connects to `ws://<jetson-ip>:8765`, and imports
  `ros2_ws/src/chassis_bringup/foxglove/drivetrain.json`. No local build or X display
  needed beyond Studio itself; one WebSocket, so it works over the radio link.

### Cross-machine ROS 2

The gamepad + teleop run on **[ground]** and publish
`/diff_drive_controller/cmd_vel_unstamped`; `diff_drive_controller` runs on **[rover]**
(inside Gazebo for sim, standalone for real). For that topic to cross, both machines must
be one DDS graph:

- **Same `ROS_DOMAIN_ID`** — pinned to `42` in `ros2_ws/pixi.toml` (`[activation.env]`), so
  every `pixi run` / `pixi shell` on both machines matches. A raw shell that skips pixi
  must `export ROS_DOMAIN_ID=42` itself. Change the value (both machines) if it clashes on
  a shared LAN / at competition.
- **Same L2 LAN**, multicast not blocked. On a trusted LAN, allow the FastDDS UDP ports
  (domain 42 ≈ 17900–18000) or drop the host firewall.
- **Default RMW** (`rmw_fastrtps_cpp`) on both — nothing to set.

Test the link from **[ground]** before plugging in the gamepad:
```bash
pixi run --environment default ros2 topic list          # must list /diff_drive_controller/*, /tf, /robot_description
pixi run --environment default ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped \
  geometry_msgs/msg/Twist '{linear: {x: 0.4}, angular: {z: 0.3}}'   # Gazebo rover on the Jetson drives a circle
```

Once this works, RViz on **[ground]** also works direct over DDS (no bridge). All rover
topics now cross DDS — fine on a LAN; for a bandwidth-limited radio link switch to a
`zenoh-bridge-ros2dds` on each side bridging only `cmd_vel` up and `odom`/`tf`/
`robot_description`/`joint_states` down (not yet set up).

### CAN backend (`can_interface`)

`sim_gz.launch.py` loads the Gazebo backend; `real.launch.py` loads
`odesc/OdescSystemHardware` (the xacro arg `use_sim` is set for you by the launch file —
not something you pass). For the real backend, `can_interface:=` picks *what the driver
talks to*:

| `can_interface` | opens a socket? | other end of the bus | use it for |
|---|---|---|---|
| **`can0`** (default) | yes — real Jetson SocketCAN | the 6 physical ODESC V4.2 + NEO controllers (ODrive CANSimple, 500 kbit/s) | driving the actual rover |
| **`vcan0`** | yes — a kernel *virtual* CAN device (`tooling/can-up vcan0`) | nothing — you `cansend` fake `Get_Encoder_Estimates` frames and `candump -L vcan0` the driver's `0x07`/`0x0D` | checking the exact CAN frames the driver emits, no hardware |
| **`mock`** / `none` | **no** | n/a — the plugin loops the commanded wheel velocity back through the 48:1 ratio and integrates position | bringing up the whole real (non-Gazebo) stack → `/odom` → TF → RViz/Foxglove with zero CAN and zero motors |

The **node ID ↔ wheel-joint map** is baked into
`robot_description/urdf/ros2_control.urdf.xacro` (per-joint `<param name="node_id">`,
0–5), canonical copy `odesc/config/node_map.yaml` — kept in sync by hand, expanded
automatically by `real.launch.py`. It is **not** a launch argument. `gear_ratio` (48:1) is
the motor-shaft-turns-per-wheel-turn conversion, applied only inside
`OdescSystemHardware`; `controllers.yaml` / `wheel_radius` / `wheel_separation` are
unaffected by it.

### Notes

- **`use_sim_time`**: sim = `true` (Gazebo `/clock`), real = `false` (wall clock).
  `real.launch.py` forces wall clock — a standalone `ros2_control_node` with
  `use_sim_time:=true` and no `/clock` wedges at "Loading controller 'diff_drive_controller'".
- **CAN bitrate 500000** matches `BILLEE_NEO_ODESC_Hardware_Integration_Guide.md` §3.4.
  Override: `tooling/can-up can0 250000`.
- **Backend / node map / `can_interface`**: see [CAN backend](#can-backend-can_interface)
  above. Short version — sim vs real is picked by the launch file, the node→wheel map is
  in the URDF (not an arg), and `can_interface:=` only chooses `can0` / `vcan0` / `mock`.
- **Teleop** (`teleop/config/joystick.yaml`, `joy_teleop.cpp`) — arcade, Xbox + PlayStation:
  `deadman_button 5` (RB/R1), `steer_axis 0` (left stick X), `throttle_axis 5` (RT/R2),
  `reverse_axis 2` (LT/L2), `steer_scale 1.5`, `speed_scale 2.0`.
  `linear.x = (fwd−rev)·speed_scale`, `angular.z = steer·steer_scale`; all-zero unless the
  deadman is held. Every index/sign is a param — retune in the yaml, no rebuild.
- **`xvfb-run`** is only for headless Gazebo on the rover. Never needed for RViz — RViz runs
  on **[ground]** with a real display.
- **RT warning** `Could not enable FIFO RT scheduling policy` from `ros2_control_node` is
  harmless (runs `SCHED_OTHER`). For low jitter grant `rtprio` via `/etc/security/limits.d/`
  or a systemd unit with `AmbientCapabilities=CAP_SYS_NICE`. Not `setcap` (breaks conda libs).

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| `colcon build` killed / machine hangs (rover) | RAM pressure (8 GB). Keep `--parallel-workers 2 --executor sequential`; `/swapfile2` (8 GB) is active. |
| Gazebo window never appears (rover, SSH) | Expected — headless. Use `xvfb-run -a`; view in RViz/Foxglove. |
| RViz empty tree / no topics ([ground]) | Not on the rover's DDS graph. Same LAN + `ROS_DOMAIN_ID` (`echo $ROS_DOMAIN_ID` → `42` on both); DDS UDP ports open. Test: `pixi run --environment default ros2 topic list`. See [Cross-machine ROS 2](#cross-machine-ros-2). |
| RViz "package 'chassis_bringup' not found" / missing meshes | `[ground]` workspace not built: `pixi run --environment default build`. |
| Gamepad on [ground] but the Gazebo rover doesn't move | `[ground]` not on the rover's DDS graph — `ros2 topic list` from `[ground]` must show `/diff_drive_controller/cmd_vel_unstamped`. Check same `ROS_DOMAIN_ID` (`42`), same LAN, firewall. Then `ros2 topic echo /joy` shows pad input, and the deadman (RB/R1) is held. |
| `joy_node` not found / teleop won't launch ([ground]) | `joy` missing from the `default` env. `pixi run --environment default ros2 pkg executables joy` should list `joy_node`; if not, add `ros-humble-joy` to `pixi.toml` `[dependencies]` and rebuild the env. |
| Foxglove "connection refused" | Bridge not running or wrong IP. `hostname -I` on rover; port 8765. |
| Foxglove: robot shows as bare axes, no meshes | Bridge not serving `package://` assets. Confirm `robot_description` is in the sourced overlay on the rover and the `foxglove_bridge` build supports asset fetch. |
| Foxglove layout imports but a display is missing | Studio schema drift — add the layer via the 3D panel settings, then Layouts → Export and overwrite `foxglove/drivetrain.json`. |
| Robot model shows but never moves (sim) | Deadman (RB/R1) not held, triggers not pulled once (they read 0 until first pull), or nothing on `/diff_drive_controller/cmd_vel_unstamped` (`ros2 topic echo` it). |
| Model sits still while the odom trail / pose drives away | The view is *following* the robot. RViz: Views → Target Frame → `odom` (or `<Fixed Frame>`). Foxglove: Frame → Follow mode → `Off`, Display frame → `odom`. Re-import `drivetrain.json` / reload `drivetrain.rviz` to pick up the checked-in fix. |
| Real: wheels dead, `list_controllers` shows `inactive` | `can0` down / wrong bitrate → `on_activate` failed. `tooling/can-up` first; check `dmesg`. No motors: `can_interface:=mock` or `vcan0`. |
| Real: hangs at "Loading controller 'diff_drive_controller'" | Standalone `ros2_control_node` on `use_sim_time:=true` with no `/clock`. `real.launch.py` handles it; if hand-rolled, pass `-p use_sim_time:=false`. |
| `can_interface:=vcan0` but no frames | `tooling/can-up vcan0` first; `candump -L vcan0`. Expect six `0x07` frames on activate, then `0x0D` per write while driving. |
| `ros2 pkg list` missing the 4 packages | Overlay not sourced — go through `pixi run --environment <env>`; confirm `install/` exists. |
| Joystick does nothing, `/joy` silent | Wrong `device_id` / not `/dev/input/js0`. `ls /dev/input/js*`, `jstest`. Chain: `/joy` → `joy_tank_drive` → `/diff_drive_controller/cmd_vel_unstamped` (hold RB/R1). |
