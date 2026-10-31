# BILLEE Drivetrain — Run Guide

Commands are bare ROS 2 commands. Run them inside `pixi shell -e <env>` on the machine tagged.
Each section has a **▸ What this does** dropdown for a refresher on how it works.

| tag | machine | env | runs |
|-----|---------|-----|------|
| **[rover]** | Jetson AGX Orin (BILLEE 2027) or Orin Nano, headless | `l4t` | sim/real bring-up, controllers, `foxglove_bridge` |
| **[ground]** | x86 Linux with monitor | `default` | gamepad teleop, RViz, Foxglove Studio |
| **[mac]** | Apple Silicon, devcontainer | `mac-cpu` | headless sim + Foxglove Studio (no RViz, gamepad, or CAN) |

---

## 1. Every terminal

```bash
cd ~/billee-software-2027/ros2_ws      # in a container: /workspaces/<clone>/ros2_ws (opens here)
pixi shell -e <env>                    # l4t | default | mac-cpu
```

<details><summary>▸ What this does</summary>

- `pixi shell` activates the env defined in `pixi.toml`: ROS 2 Humble (RoboStack), build tools,
  `ROS_DOMAIN_ID=42`, `FPRIME_GDS_URL`, and it sources `install/setup.sh` so your built packages
  are found.
- One-off alternative to the shell: `pixi run -e <env> <cmd>`.
- Outside a container, pixi may need `export PATH="$HOME/.pixi/bin:$PATH"`.
- A `Could not find activation scripts: .../install/setup.sh` warning = workspace not built yet (§3).
</details>

---

## 2. Setup (once per machine)

| platform | host | container (VS Code → **Dev Containers: Reopen in Container**) | env | good for |
|---|---|---|---|---|
| **Linux x86 + NVIDIA** | Ubuntu, NVIDIA driver for CUDA 13.2 | `desktop-roshumble_dev-x862` · or `tooling/desktop-ros2` | `default` | everything: GPU sim, RViz, gamepad, ZED, CAN |
| **Jetson AGX Orin / Orin Nano** (rover) | JetPack 6.0 (L4T r36.3) on both | `rover-roshumble_l4t-aarch64` · or `tooling/rover-ros2` | `l4t` | rover: real drivetrain, ZED |
| **Apple Silicon Mac** | macOS + Docker Desktop / Rancher Desktop | `desktop-roshumble-mac-cpu` | `mac-cpu` | headless sim + Foxglove |
| **Native pixi** | Linux x86 (CUDA 13) or aarch64 | none | `default` / `l4t` | quick tools; no ZED SDK |

**Every platform:**
```bash
git clone https://github.com/BILLEE-BroncoSTAR/billee-software-2027.git ~/billee-software-2027
```
Install VS Code with the **Remote Development** extension pack, plus **Foxglove Studio** on
whatever machine you view from. After the container opens: `pixi shell -e <env>`, then build (§3).
In VS Code, set the Python interpreter to `ros2_ws/.pixi/envs/<env>/bin/python3`.

<details><summary>▸ What the containers have in common</summary>

- The repo is mounted at `/workspaces/<clone>` (git works inside) and the shell opens in `ros2_ws`.
- `.bashrc` (from `docker/terminal_config.sh`) sources the pixi env named by `BILLEE_PIXI_ENV`
  (`mac-cpu`/`l4t`, else `default`) and `install/setup.bash`, so `ros2` works right away.
- ROS 2 comes from pixi (RoboStack), not apt — the image only provides the OS, CUDA, ZED SDK
  and pixi itself. `ros2_ws/.pixi` lives in your clone, so it survives container rebuilds.
- Pixi envs aren't relocatable: if the mount path changes, the env reinstalls on first start.
- Rebuild the container only when a Dockerfile or `devcontainer.json` changes.
</details>

### 2.1 Linux x86 + NVIDIA (ground station / main dev box)

```bash
# Docker Engine: https://docs.docker.com/engine/install/ubuntu/
sudo usermod -aG docker "$USER"                       # then log out / in
# NVIDIA driver (CUDA 13.2-capable) + NVIDIA Container Toolkit (add NVIDIA's apt repo first):
sudo apt install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
docker run --rm --gpus all ubuntu nvidia-smi          # smoke test: prints your GPU
```
Open the repo in VS Code → **Reopen in Container** → `desktop-roshumble_dev-x862`.
Without VS Code:
```bash
tooling/desktop-ros2 build        # build image zed-ros2-desktop:latest
tooling/desktop-ros2 shell        # bash inside it, same mounts/devices as the devcontainer
```

<details><summary>▸ What this does</summary>

- Image `docker/Dockerfile.desktop.humble`: CUDA 13.2 + Ubuntu 22.04, ZED SDK 5.4, `can-utils`,
  pixi. Runs as `ros_user`.
- `--gpus all` (needs the Container Toolkit) gives Gazebo/RViz/ZED the GPU.
- `--network=host`: DDS reaches the rover directly, and host `vcan0`/`can0` are visible.
- `/dev` is mounted: gamepad (`/dev/input/js0`), ZED (USB), CAN adapters.
- `xhost +local:` (run automatically by the devcontainer) + `/tmp/.X11-unix` lets Gazebo/RViz
  windows open on your desktop. With `desktop-ros2`, run `xhost +local:` yourself first.
- On create/update it runs `pixi install` (env `default`).
</details>

### 2.2 Jetson AGX Orin / Orin Nano (rover)

One image serves both boards. Flash both with **JetPack 6.0**
(`cat /etc/nv_tegra_release` → `R36 (release), REVISION: 3.x`).

Host prep, once per board — **done on the Orin Nano; repeat on the AGX Orin**:
```bash
sudo apt install -y nvidia-container curl     # nvidia-container-toolkit (JetPack meta-package)
curl https://get.docker.com | sh && sudo systemctl --now enable docker
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
# GPU at build time too: /etc/docker/daemon.json -> "default-runtime": "nvidia"
sudo usermod -aG docker "$USER"               # then log out / in
docker run --rm --runtime nvidia ubuntu:24.04 nvidia-smi
# real CAN bus, persisted across reboots:
sudo cp tooling/can0.service /etc/systemd/system/ && sudo systemctl enable --now can0.service
```
Open in VS Code (over Remote-SSH) → **Reopen in Container** → `rover-roshumble_l4t-aarch64`.
Without VS Code:
```bash
tooling/rover-ros2 build                                  # image rover-ros2:latest
tooling/rover-ros2 build --build-arg L4T_VERSION=r36.4.0  # different JetPack (ZED SDK follows)
tooling/rover-ros2 shell
```
Build: AGX Orin → normal `pixi run -e l4t build`; Orin Nano (8 GB) → the low-RAM command in §3.

<details><summary>▸ What this does</summary>

- Image `docker/Dockerfile.l4t-humble`: `nvcr.io/nvidia/l4t-jetpack:${L4T_VERSION}` (default
  `r36.3.0` = JetPack 6.0), ZED SDK 5.2 (newest with an L4T 36.3 build; the download URL is
  derived from `L4T_VERSION`), pixi. Runs as `ros_user`.
- Board-agnostic: the JetPack base image and the ZED SDK `.run` are keyed to the L4T version,
  not the board, so AGX Orin and Orin Nano use the same image as long as both run the same L4T.
  A board on a different JetPack builds with `--build-arg L4T_VERSION=` matching its release.
- `--runtime nvidia` (Jetson's GPU runtime), not `--gpus all`.
- Host network + `/dev` → `can0`, ZED and DDS to the ground station all work; hostname `rover`.
- `can0` is a host interface: bring it up on the Jetson host (`tooling/can-up` or the service),
  never inside the container.
- For a different JetPack in the devcontainer, add `"args": {"L4T_VERSION": "..."}` under
  `build` in `.devcontainer/l4t/devcontainer.json`.
- On start it runs `pixi install --environment l4t`.
</details>

### 2.3 Apple Silicon Mac

1. Install Docker (**Docker Desktop** or **Rancher Desktop**), VS Code + Remote Development,
   and **Foxglove Studio**.
2. Open the repo → **Reopen in Container** → `desktop-roshumble-mac-cpu`.
3. `pixi shell -e mac-cpu`, build (§3). View in Foxglove → `ws://localhost:8765`.

<details><summary>▸ What this does</summary>

- Image `docker/Dockerfile.desktop.mac.humble`: plain Ubuntu 22.04 arm64, xvfb + mesa
  (software OpenGL for headless Gazebo), pixi. Runs as `root`.
- `mac-cpu` uses the same packages as the Jetson's `l4t` env, so builds behave the same.
- Port 8765 is forwarded to the Mac (Docker Desktop has no host networking).
- **Not available:** GPU, ZED SDK, USB gamepad, `vcan`/CAN, DDS to a remote rover. Drive with
  keyboard teleop or Foxglove's Teleop panel; reach a rover via `ws://<rover-ip>:8765`.
- Native (no container) isn't possible: `pixi.toml` only targets Linux.
</details>

### 2.4 Native pixi (Linux, no Docker)

```bash
curl -fsSL https://pixi.sh/install.sh | bash          # then open a new terminal
cd ~/billee-software-2027/ros2_ws
pixi install -e default                               # x86 + CUDA 13; use l4t on aarch64
```

<details><summary>▸ What this does</summary>

- Installs ROS 2 Humble and every workspace dependency into `ros2_ws/.pixi` — nothing system-wide.
- You get only what `pixi.toml` lists: no ZED SDK, `can-utils` or xvfb. Install those from apt
  if you need them, or use a container.
- `default` requires a CUDA 13 driver on the host (the `linux-64-cuda` platform).
</details>

---

## 3. Build

```bash
pixi run -e <env> build
ros2 pkg list | grep -E 'chassis_bringup|odesc|robot_description|teleop'   # all four print
```

<details><summary>▸ What this does</summary>

- `build` is a task in `pixi.toml` → `colcon build --symlink-install --base-paths src`.
- colcon builds every package in `src/` in dependency order into `build/` (scratch),
  `install/` (what ROS runs), `log/`. None are in git — each machine builds its own.
- `--symlink-install` links Python/launch/YAML, so edits to those apply without rebuilding.
- **Rebuild when:** C++/`CMakeLists.txt` changes, or new packages/files. Then open a new shell.
- One package: `pixi run -e <env> build --packages-select <pkg>`.
- Orin Nano low on RAM (8 GB; AGX Orin doesn't need this): `colcon build --symlink-install --parallel-workers 2 --executor sequential --base-paths src --cmake-args ' -DCMAKE_BUILD_TYPE=Release'`.
</details>

---

## 4. Quick launch

```bash
# [rover] sim (Gazebo, no ODESC)
xvfb-run -a ros2 launch chassis_bringup rover.launch.py
# [rover] real drivetrain (bring up can0 first, §6)
ros2 launch chassis_bringup rover.launch.py mode:=real        # + can_interface:= gear_ratio:=

# [ground]
ros2 launch chassis_bringup ground_station.launch.py fprime_gds:=true   # use_sim_time:=false on real

# [mac] sim, then drive from a second terminal
xvfb-run -a ros2 launch chassis_bringup rover.launch.py mode:=sim
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

View: Foxglove Studio → `ws://localhost:8765` (or `ws://<rover-ip>:8765`) → import
`ros2_ws/src/chassis_bringup/foxglove/drivetrain.json`.

<details><summary>▸ What this does</summary>

- `rover.launch.py` = robot + controllers on the chosen backend (Gazebo or ODESC CAN) +
  `foxglove_bridge` on :8765. No viewer or teleop.
- `ground_station.launch.py` = joystick teleop + RViz + a local `foxglove_bridge`.
- `xvfb-run -a` gives Gazebo a fake display on headless machines. Never needed for RViz.
- `fprime_gds:=true` opens the F´ GDS web UI (needs a real browser/desktop). URL priority:
  `fprime_gds_url:=` arg → `$FPRIME_GDS_URL` → launch default. Team default lives in
  `pixi.toml` `[activation.env]`.
- `use_sim_time`: `true` in sim (Gazebo `/clock`), `false` on real hardware.
- Mac/remote rover: no DDS across Docker Desktop — drive from Foxglove's Teleop panel instead.
</details>

---

## 5. Simulation, step by step

```bash
# [rover] 1. Gazebo + robot + controllers
xvfb-run -a ros2 launch chassis_bringup sim_gz.launch.py
# [rover] 2. Foxglove bridge
ros2 launch chassis_bringup viz.launch.py rviz:=false use_sim_time:=true
# [ground] 3. RViz
ros2 launch chassis_bringup viz.launch.py rviz:=true foxglove:=false use_sim_time:=true
# [ground] 4. gamepad
ros2 launch teleop teleop.launch.py
```

<details><summary>▸ What this does</summary>

- `sim_gz.launch.py` starts Gazebo Fortress (`world_file:=`, default `empty.sdf`; `headless:=true`
  = server only, no GUI), spawns the URDF with `use_sim:=true`
  (backend `ign_ros2_control`), loads `diff_drive_controller` + `joint_state_broadcaster`, and
  bridges `/clock` plus the depth camera (`/depth_cam/*`) and `/gz/odom` (`config/zed_config.yaml`).
- `viz.launch.py` has two switches: `rviz:=` and `foxglove:=`. Both views read
  `/robot_description`, `/tf`, `/diff_drive_controller/odom`.
- Teleop publishes `/diff_drive_controller/cmd_vel_unstamped`; it reaches the rover over DDS
  (same LAN + `ROS_DOMAIN_ID`, §9). Teleop uses wall-clock time, not `/clock`.
</details>

### ODESC shadow test (Gazebo drives, ODESC path exercised on vCAN)

```bash
~/billee-software-2027/tooling/can-up vcan0          # Linux host, sudo, idempotent
xvfb-run -a ros2 launch chassis_bringup sim_gz.launch.py odesc_shadow:=true can_interface:=vcan0
candump -L vcan0                                     # watch the frames (can-utils)
~/billee-software-2027/tooling/can-up down vcan0     # teardown
```

<details><summary>▸ What this does</summary>

- A second controller manager under `/odesc_shadow/` gets the same `cmd_vel` as Gazebo and
  drives `OdescSystemHardware` on `vcan0` — tests teleop/Nav2 → diff drive → CAN frames with no
  motors. Its TF is off so Gazebo stays authoritative.
- A built-in emulator turns `Set_Input_Vel` frames into `Get_Encoder_Estimates` for nodes 0–5.
- `vcan` is a host-kernel module: create it on the Linux **host**, not in the container (Linux
  devcontainers use `--network=host`, so it's visible inside). Not possible on Mac.
</details>

---

## 6. Real drivetrain (ODESC over CAN)

```bash
# [rover]
~/billee-software-2027/tooling/can-up                       # can0 @ 500 kbit/s
~/billee-software-2027/tooling/can-up status can0
ros2 launch chassis_bringup real.launch.py                  # can_interface:= gear_ratio:= foxglove:=
ros2 launch chassis_bringup viz.launch.py rviz:=false use_sim_time:=false
ros2 control list_hardware_components                      # 'Robot' ACTIVE
ros2 control list_controllers                              # both active
# [ground]
ros2 launch chassis_bringup viz.launch.py rviz:=true foxglove:=false use_sim_time:=false
ros2 launch teleop teleop.launch.py
```

No hardware: `real.launch.py can_interface:=mock` (no CAN at all) or `can_interface:=vcan0`.

<details><summary>▸ What this does</summary>

- `can-up` configures SocketCAN `can0` at 500000 (matches the ODESC guide §3.4; override:
  `can-up can0 250000`). Persist: `sudo cp tooling/can0.service /etc/systemd/system/ && sudo systemctl enable --now can0.service`.
- `real.launch.py` sets `use_sim:=false` → loads `odesc/OdescSystemHardware` and forces wall-clock time.

| arg | default | meaning |
|---|---|---|
| `can_interface` | `can0` | `can0` real bus · `vcan0` virtual bus · `mock` loopback, no socket |
| `gear_ratio` | `48.0` | motor turns per wheel turn (only used inside the ODESC plugin) |
| `foxglove` | `false` | `true` folds in the bridge |

- `mock` loops commanded wheel velocity back as feedback → `/odom`, TF, viewers all work.
- `vcan0`: `cansend` fake encoder frames, `candump -L vcan0` to see the driver's `0x07`/`0x0D`.
- CAN node ↔ wheel map is in `robot_description/urdf/ros2_control.urdf.xacro` (canonical copy
  `odesc/config/node_map.yaml`, synced by hand). Not a launch arg.
- `rover.launch.py mode:=real` is the one-command version of the above.
</details>

---

## 7. Driving

Gamepad: hold **RB/R1** (deadman) · **RT/R2** forward · **LT/L2** reverse · **left stick** steer.
Release RB = stop. Without a gamepad (also a link test):

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.4}, angular: {z: 0.3}}'
```

<details><summary>▸ What this does</summary>

- Chain: `/dev/input/js0` → `joy_node` → `/joy` → `joy_tank_drive` → `cmd_vel_unstamped` →
  `diff_drive_controller` → wheels.
- `teleop/config/joystick.yaml`: deadman 5, steer axis 0, throttle 5, reverse 2,
  `steer_scale 1.5`, `speed_scale 2.0`. `linear.x = (fwd−rev)·speed_scale`,
  `angular.z = steer·steer_scale`, zero unless deadman held. Retune in YAML, no rebuild.
- Triggers read 0 until first pulled.
- `topic pub -r 10` publishes at 10 Hz → the rover drives a circle.
</details>

---

## 8. People / pose (OpenPose, sim)

```bash
src/perception/people/scripts/get_openpose_models.sh          # once: ~200 MB into ros2_ws/models/
pixi run -e <env> build --packages-select robot_description chassis_bringup people
# terminal 1 — sim with one standing person 4 m ahead (or worlds/people_walking.sdf)
xvfb-run -a ros2 launch chassis_bringup sim_gz.launch.py world_file:=worlds/people.sdf navigation:=true foxglove:=true headless:=true
# terminal 2 — pose (wait for terminal 1 to settle)
ros2 launch people people.launch.py net_height:=256
```
Foxglove → `ws://localhost:8765` → import `ros2_ws/src/perception/people/foxglove/people.json`
(left: skeleton on the camera image; right: 3D skeleton in `odom`). On a Mac CPU the first
skeleton takes ~10 s and then updates every few seconds — each one stays up until the next.

Record once, then iterate without Gazebo:
```bash
ros2 bag record -o people_run1 /depth_cam/image /depth_cam/depth_image /depth_cam/camera_info /tf /tf_static /gz/odom /clock /robot_description
ros2 bag play people_run1 --clock        # + people.launch.py; Foxglove can also open the bag directly
```
Tests: `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest src/perception/people/test`

<details><summary>▸ What this does</summary>

- `openpose_node` syncs the camera's colour image, depth image and `camera_info`, runs the
  OpenPose COCO model (OpenCV DNN, CPU), takes the strongest peak per joint (**one person**),
  reads the depth under each joint, back-projects it into `camera_link_optical`, and transforms
  it into `odom` at the image's timestamp — i.e. where the **rover believes** its camera is.
- Publishes `/people/debug_image` (2D skeleton) and `/people/markers` (red 3D skeleton).
- Joints whose depth is far from the body's median are dropped (thin limbs can hit background).
- `navigation:=true` is required: the EKF owns `odom → base_link` (diff-drive TF is off).
- Keypoints are COCO-18 — the same order as ZED `BODY_18` and `trt_pose`, so a faster backend
  can replace the model later without changing the 3D/TF part.
- Weights come from a pinned mirror (CMU's server is offline) and are checksum-verified.
- Bags: ~64 MB/s at 1280×720 RGB + depth — keep clips short. Pytest needs
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` because the env's `launch_testing` plugin breaks pytest.
- The Gazebo people models download from Gazebo Fuel on first launch (needs internet).
- `headless:=true` runs only the Gazebo server (sensors still render under `xvfb-run`); the
  hidden GUI otherwise eats about half the CPU OpenPose needs.
- `net_height` is OpenPose's input height: 368 ≈ 4.7 s/frame, 256 ≈ 2.3 s/frame on a Mac CPU.
- TF is one chain: `odom → base_footprint` (EKF) `→ base_link` (URDF) `→ camera_link → camera_link_optical`.
  The EKF is planar (`two_d_mode`), fuses wheel speed + yaw rate and synthetic-VIO x, y, yaw.
- The skeleton's few-degree heading / sideways error is the synthetic VIO's deliberate noise —
  the only absolute heading source in sim. Its yaw drift is a random walk that grows with time,
  **even when parked**. Default `profile: calibration` (nominal error) in
  `zed2i/config/synthetic_vio.yaml`; `validation` (up to 2× error) reached ~13° in 5 min — use it
  only for EKF evaluation runs.
- Run **one sim at a time** per container/machine: two sims share ROS and Gazebo topics (two
  clocks, two `/gz/odom`), which corrupts TF and can tip or freeze the rover.
</details>

---

## 9. Reference

<details><summary>▸ Cross-machine ROS 2 (ground ↔ rover)</summary>

- Same `ROS_DOMAIN_ID` (42, set in `pixi.toml`; a raw shell must `export ROS_DOMAIN_ID=42`).
- Same L2 LAN, multicast allowed; open FastDDS UDP (~17900–18000 for domain 42).
- Default RMW `rmw_fastrtps_cpp` on both.
- Test from [ground]: `ros2 topic list` must show `/diff_drive_controller/*`, `/tf`.
- For a radio link, plan: `zenoh-bridge-ros2dds` bridging only `cmd_vel` up and
  `odom`/`tf`/`joint_states` down (not set up yet).
</details>

<details><summary>▸ RViz vs Foxglove</summary>

- **RViz**: on [ground], needs a display and the `default` workspace built locally.
- **Foxglove**: rover runs only the bridge; Studio connects over one WebSocket — works over
  radio, needs no local build. Both can run at once.
</details>

<details><summary>▸ Real-time warning</summary>

`Could not enable FIFO RT scheduling policy` is harmless. For low jitter, grant `rtprio` via
`/etc/security/limits.d/` or a systemd unit with `AmbientCapabilities=CAP_SYS_NICE` — not
`setcap` (breaks conda libs).
</details>

---

## 10. Troubleshooting

| Symptom | Fix |
|---|---|
| `package not found` / missing meshes | Not built or not sourced: build (§3), open a new `pixi shell`. |
| `colcon build` killed (Orin Nano) | RAM: use the low-RAM build in §3. |
| Gazebo window never appears | Expected headless. Use `xvfb-run -a`, view in RViz/Foxglove. |
| RViz empty / gamepad doesn't move rover | Not on the rover's DDS graph — §9. Then `ros2 topic echo /joy` and hold RB. |
| `joy_node` not found | `ros2 pkg executables joy`; if empty add `ros-humble-joy` to `pixi.toml`. |
| `/joy` silent | `ls /dev/input/js*`, `jstest`; check `device_id`. |
| Foxglove "connection refused" | Bridge not running or wrong IP (`hostname -I`), port 8765. |
| Foxglove shows bare axes | Bridge can't serve `package://` meshes — `robot_description` not in the rover's overlay. |
| Foxglove rover on its side (wheels flat, chassis upright) but TF axes upright | 3D panel → Scene → **Mesh up-axis: Z-up** (STL meshes; Y-up is Foxglove's default). Both layout files set it. |
| Model still, odom trail moves | Expected: view follows `base_link`. Set fixed/follow frame to `odom` / Off. |
| Real: controllers `inactive` | `can0` down or wrong bitrate → `can-up`, check `dmesg`. |
| Real: hangs "Loading controller" | `use_sim_time:=true` with no `/clock`. Use `real.launch.py` or `-p use_sim_time:=false`. |
| `vcan0`: no frames | `can-up vcan0` first; expect six `0x07` on activate, `0x0D` while driving. |
| People: no / very slow skeletons | `pgrep -af "ign gazebo"` — only one sim may run (a stray one fights for topics and CPU; `pkill -9 -f "ign gazebo"`). Use `headless:=true`, `net_height:=256`. Mac: `top` — if `cpptools` is high, **Developer: Reload Window** (limits in `ros2_ws/.vscode/settings.json`) or give Docker/Rancher more RAM. |
| People world: `Failed to configure controller`, no `odom` | First launch stalls Gazebo while it downloads the person model. Relaunch — it's cached now. |
| Rover tipped over / frozen, `jump back in time` warnings | Two sims running at once — `pgrep -af "ign gazebo"`; keep one. |
| Skeleton a few degrees / ~0.3 m off sideways | Expected: the rover's believed heading carries the synthetic VIO's noise, which grows the longer the sim runs (§8). Check `ros2 param get /synthetic_vio profile` is `calibration`; restart the sim to reset the drift. |
