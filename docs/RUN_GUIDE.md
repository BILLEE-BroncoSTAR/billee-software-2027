# BILLEE Drivetrain — Run Guide

Commands are bare ROS 2 commands. Run them inside `pixi shell -e <env>` on the machine tagged.
Each section has a **▸ What this does** dropdown for a refresher on how it works.

| tag | machine | env | runs |
|-----|---------|-----|------|
| **[rover]** | Jetson AGX Orin (BILLEE 2027) or Orin Nano, headless | `l4t` | sim/real bring-up, controllers, `foxglove_bridge`, zed-openpose (§8) |
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
| **Jetson AGX Orin / Orin Nano** (rover) | JetPack 7.2 (L4T r39.2) on both | `rover-roshumble_l4t-aarch64` · or `tooling/rover-ros2` | `l4t` | rover: real drivetrain, ZED |
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

One image serves both boards. Flash both with **JetPack 7.2**
(`cat /etc/nv_tegra_release` → `R39 (release), REVISION: 2.x`).

Host prep, once per board — **done on the Orin Nano; repeat on the AGX Orin**:
```bash
sudo apt install -y nvidia-container curl     # nvidia-container-toolkit (JetPack meta-package)
curl https://get.docker.com | sh && sudo systemctl --now enable docker
sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker
# GPU at build time too: /etc/docker/daemon.json -> "default-runtime": "nvidia"
sudo usermod -aG docker "$USER"               # then log out / in
ls /var/run/cdi/nvidia.yaml                   # GPU CDI spec (JetPack 7 generates it); else:
#   sudo nvidia-ctk cdi generate --output=/var/run/cdi/nvidia.yaml
# ZED 2i USB access for the container's non-root user (the rule ships in the image):
docker run --rm rover-ros2 cat /etc/udev/rules.d/99-slabs.rules | sudo tee /etc/udev/rules.d/99-slabs.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
# real CAN bus, persisted across reboots:
sudo cp tooling/can0.service /etc/systemd/system/ && sudo systemctl enable --now can0.service
```
Open in VS Code (over Remote-SSH) → **Reopen in Container** → `rover-roshumble_l4t-aarch64`.
Without VS Code:
```bash
tooling/rover-ros2 build                                  # image rover-ros2:latest
tooling/rover-ros2 build --build-arg L4T_VERSION=r39.2 --build-arg CUDA_VERSION=13-2  # other JetPack 7.x
tooling/rover-ros2 shell
```
Build: AGX Orin → normal `pixi run -e l4t build`; Orin Nano (8 GB) → the low-RAM command in §3.

<details><summary>▸ What this does</summary>

- Image `docker/Dockerfile.l4t-humble`: `ubuntu:24.04` + CUDA 13.2, cuDNN 9 and the TensorRT
  runtime from NVIDIA's Jetson apt repo (`L4T_VERSION`, default `r39.2` = JetPack 7.2; NVIDIA
  ships no `l4t-jetpack` image for JetPack 7), ZED SDK 5.5 (newest with an L4T 39.2 build; the
  download URL is derived from `L4T_VERSION`), OpenPose in `/opt/openpose`, pixi. Runs as `ros_user`.
- Board-agnostic: the Jetson apt packages and the ZED SDK `.run` are keyed to the L4T version,
  not the board, so AGX Orin and Orin Nano use the same image as long as both run the same L4T.
  A board on a different JetPack builds with `--build-arg L4T_VERSION=` matching its release.
- `--runtime nvidia` (Jetson's GPU runtime), not `--gpus all`, plus `--device nvidia.com/gpu=all`
  (CDI): it adds the host's `video` and `render` gids, which CUDA needs as the non-root
  `ros_user` (`CUDA error 801` / `no CUDA-capable device` otherwise).
- Host network + `/dev` → `can0`, ZED and DDS to the ground station all work; hostname `rover`.
- `can0` is a host interface: bring it up on the Jetson host (`tooling/can-up` or the service),
  never inside the container.
- For a different JetPack in the devcontainer, add `"L4T_VERSION"` / `"CUDA_VERSION"` to
  `build.args` in `.devcontainer/l4t/devcontainer.json`.
- Disk: the image is ~10 GB; leave ~20 GB free for the first build (pixi env + OpenPose build
  tree). On a small SD card, Docker's classic store saves a few GB:
  `/etc/docker/daemon.json` → `"features": {"containerd-snapshotter": false}`.
- OpenPose compiles with all cores; if nvcc runs out of RAM, `export OPENPOSE_JOBS=2` on the
  host before **Rebuild Container**.
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
- `xvfb-run -a` gives Gazebo a fake display on headless machines (Mac, Jetson). The x86
  devcontainer has no `xvfb-run`: it uses the host's display and GPU, so drop the prefix there.
  Never needed for RViz.
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

## 8. People / pose (zed-openpose on the rover)

A ROS 2 port of [stereolabs/zed-openpose](https://github.com/stereolabs/zed-openpose): CMU OpenPose
runs on the Jetson's GPU (CUDA + cuDNN) on the ZED 2's left image, and each keypoint is placed in 3D
from the ZED's depth. Rover only, with the ZED plugged in; the ground station just views it.

```bash
# [rover] once: OpenPose models (~300 MB into ros2_ws/models/openpose; CMU's server is offline)
src/perception/zed_openpose/scripts/get_openpose_models.sh
pixi run -e l4t build --packages-select people_msgs zed_openpose people robot_description
# [rover] ZED + OpenPose + gestures + robot_state_publisher + foxglove_bridge
ros2 launch people people_rover.launch.py
#   …next to the drivetrain (real.launch.py foxglove:=true): add standalone:=false
#   …replay a ZED recording instead of the camera: svo_path:=/path/rec.svo2
# [ground] Foxglove → ws://<rover-ip>:8765 → import ros2_ws/src/perception/people/foxglove/people_rover.json
#          (RViz: fixed frame base_link, MarkerArray /people/markers, Image /people/debug_image)
```

<details><summary>▸ What this does</summary>

- `zed_openpose_node` (package `zed_openpose`) keeps the sample's pipeline: a ZED thread grabs the
  left image + XYZ point cloud at 720×405 while an OpenPose thread runs the previous frame through
  `op::PoseExtractorCaffe` (BODY_25, `net_resolution` 320x240) on the GPU. Every joint takes the
  XYZ under its pixel, else the nearest valid one within 10 px; a limb is drawn only if shorter
  than 1 m and within 1.8 m of the person's centre. Multi-person.
- The sample's windows became topics: `/people/markers` (3D skeletons, its OpenGL view),
  `/people/debug_image` + `/compressed` (OpenPose's rendering, its OpenCV view), optional
  `/zed/points` (`publish_cloud:=true`) and `/zed/depth_image` (`publish_depth:=true`).
  `/people/skeletons` (`people_msgs/Skeletons`) carries every person's 2D + 3D joints.
- Placement: the ZED reports points in its left lens frame with ROS axes; the node adds the
  static TF `camera_link → zed_left_camera_frame` (half a baseline to +y) and transforms into
  `base_link`, so skeletons are **relative to the rover**. The sample's floor-plane alignment is
  replaced by the URDF mount: measure the real ZED mount and set it in
  `robot_description/urdf/camera.urdf.xacro`, or every skeleton is off by the difference.
- The sample's flags are ROS params with the same names: `zed_openpose/config/zed_openpose.yaml`.
- Build: `docker/Dockerfile.l4t-humble` compiles OpenPose with CUDA/cuDNN for the Orin (sm_87)
  into `/opt/openpose` (needs the nvidia default runtime at build time, see the README). Off the
  Jetson, `zed_openpose` builds only its unit test and skips the node.
- No ZED plugged in → the node exits with `ZED open failed`. No sim or CPU fallback.
</details>

### 8.1 Gestures

`people_rover.launch.py` also starts `gesture_node`. It takes the person closest to the rover from
`/people/skeletons` and publishes `/people/gesture` (`people_msgs/Gesture`) once per frame:

| Gesture | Pose | Default `command` |
|---|---|---|
| `arms_u` | both arms up in a U: upper arms out + forearms up, or arms straight up | `stop` |
| `t_pose` | both arms straight out to the sides | `come_here` |
| `right_arm_up` | the person's right hand above their head, left arm down | `follow_me` |
| `left_arm_up` | the person's left hand above their head, right arm down | `resume` |

```bash
ros2 topic echo /people/gesture                  # gesture, command, confidence, position (base_link)
```

<details><summary>▸ What this does</summary>

- Rules on the **2D** keypoints, in shoulder widths from the neck (any distance, no depth needed);
  "right/left" are the person's own sides, also when facing away. Works for BODY_25 and COCO.
- A gesture (or going back to `none`) is reported after `hold_frames` (2) frames in a row.
- Commands live in `people/config/gestures.yaml` (placeholders); a navigation node subscribes to
  `/people/gesture` and acts on `command` and `position`.
- `/people/gesture_marker`: the gesture as yellow text above the person in Foxglove.
- New gestures: a rule + a test in `people/gestures.py` / `test/test_gestures.py`, a constant in
  `people_msgs/msg/Gesture.msg`, and a `commands.<name>` line in `gestures.yaml`.
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
| `xvfb-run: No such file or directory` (x86 devcontainer) | Drop `xvfb-run -a`; the container renders on the host display. |
| Gazebo segfaults: `unable to find OpenGL 3+ Rendering Subsystem` (x86) | `OGRE2_RESOURCE_PATH` unset — run inside `pixi shell`/`pixi run` (set in `pixi.toml`), or `export OGRE2_RESOURCE_PATH=$CONDA_PREFIX/lib/OGRE-Next`. |
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
| Rover tipped over / frozen, `jump back in time` warnings | Two sims running at once — `pgrep -af "ign gazebo"`; keep one. |
| Sim: `odom` heading a few degrees off, growing while parked | Expected: the synthetic VIO's deliberate yaw drift (a random walk). Check `ros2 param get /synthetic_vio profile` is `calibration` (`validation` = up to 2× error); restart the sim to reset it. |
| `zed_openpose`: `ZED open failed: CAMERA NOT DETECTED` | ZED not plugged in (USB 3 port) or not visible in the container: `lsusb \| grep 2b03` on host and in the container. |
| `zed_openpose` build: "skipping the node" on the Jetson | Image built without OpenPose: rebuild the l4t image with the nvidia default runtime (README → Jetson host prep); check `ls /opt/openpose/lib`. |
| `OpenPose: ... prototxt` / model not found | Run `src/perception/zed_openpose/scripts/get_openpose_models.sh` (inside `pixi shell -e l4t`). |
| Debug image but no 3D skeleton, `No TF base_link <- zed_left_camera_frame` | No `robot_state_publisher`: use `standalone:=true`, or run `real.launch.py` alongside. |
