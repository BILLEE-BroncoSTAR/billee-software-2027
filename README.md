# BILLEE rover software (URC 2027)

ROS 2 Humble workspace for the BILLEE rover: a six-wheel differential drive on ODESC motor
controllers over CAN, a Gazebo simulation of it, gamepad teleop, and ZED 2i perception.
Dependencies are managed by [pixi](https://pixi.sh) and everything runs in a dev container.

## 1. Open the dev container

Pick the configuration for your machine (VS Code: **Dev Containers: Reopen in Container**):

| Machine | Container | Pixi env |
|---|---|---|
| x86 Linux with an NVIDIA GPU (dev box, ground station) | `desktop-roshumble_dev-x862` (`.devcontainer/devcontainer.json`) | `default` |
| Jetson Orin, JetPack 6.2 / L4T r36.4 (the rover) | `rover-roshumble_l4t-aarch64` (`.devcontainer/l4t/`) | `l4t` |
| Apple-Silicon Mac (headless sim and viewer, no GPU) | `desktop-roshumble-mac-cpu` (`.devcontainer/mac/`) | `mac-cpu` |

Host requirements:

- **x86:** Docker, NVIDIA driver, NVIDIA Container Toolkit, an X session (the container runs
  `xhost +local:` on start). Check the driver with `nvidia-smi`, then install the toolkit once:

  ```sh
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
  sudo apt update && sudo apt install -y nvidia-container-toolkit
  sudo nvidia-ctk runtime configure --runtime=docker
  sudo nvidia-ctk cdi generate --output=/etc/cdi/nvidia.yaml   # re-run after a driver upgrade
  sudo systemctl restart docker
  docker run --rm --gpus all ubuntu:22.04 nvidia-smi           # should list your GPU
  ```

  Without this, the image builds but the container fails to start with
  `failed to discover GPU vendor from CDI: no known GPU vendor found`.
- **Mac:** Docker Desktop.
- **Jetson:** Docker with the `nvidia` runtime. Do this once, and again after every reflash:

  ```sh
  # nvidia-container installs Docker CE + the nvidia runtime in the background (1-2 min).
  # Don't install Docker yourself first; a leftover docker.sources file breaks that step.
  sudo rm -f /etc/apt/sources.list.d/docker.sources /etc/apt/keyrings/docker.asc
  sudo apt install -y nvidia-container
  journalctl -fu nv-install-docker        # wait for "Docker service installed", then Ctrl-C
  sudo usermod -aG docker "$USER"         # log out and back in
  docker run --rm --runtime nvidia -e NVIDIA_VISIBLE_DEVICES=all ubuntu:22.04 nvidia-smi
  ```

  Without this, the container fails to start with `unknown or invalid runtime name: nvidia`.
  The first image build pulls a ~12.6 GB base and compiles OpenPose.

Without VS Code, `tooling/desktop-ros2` (x86) and `tooling/rover-ros2` (Jetson) take
`build|run|shell` to build and run the same images.

**Or with `make`:** `make setup <mac|linux-aarch64|l4t|x86>` once per machine, then
`make build`, `make shell` and `make sim` (`make help` lists them). Per-platform walkthrough:
[docs/setup.md](docs/setup.md). The same commands, plus every run mode, are buttons in a
browser control panel: `devtool/billee-dev` ([devtool/README.md](devtool/README.md)).

## 2. Build (on every machine you use)

Each machine builds its own workspace: the Jetson, the ground station, or both. Everything runs
from `ros2_ws`. Enter the environment once per terminal, then use plain `colcon` and `ros2`
commands:

```bash
cd ros2_ws
pixi shell -e default        # x86 ground station / dev box. Jetson: -e l4t. Mac: -e mac-cpu
pixi run build               # inside the shell: builds with the active env
```

After the **first** build, or any build that adds a package, run
`source install/local_setup.bash` in every shell that was already open, or reopen it.
Otherwise `ros2 launch` fails with `Package 'chassis_bringup' not found`. To run a single command without entering the shell, use `pixi run -e <env> <cmd>`.

On an 8 GB Jetson, a full build can run out of RAM. Use
`pixi run build --parallel-workers 2 --executor sequential`.

## 3. Run

The rover's real drivetrain on the **Jetson**, driven from the **ground station**. First-time
setup of both machines (clone, container, build, OpenPose models) is in
[docs/running.md → First-time setup](docs/running.md#first-time-setup).

| Where | Command |
|---|---|
| Jetson host, once per boot | `cd ~/billee-software-2027 && tooling/can-up` |
| Jetson container | `ros2 launch chassis_bringup rover.launch.py mode:=real camera:=<none\|cloud\|openpose\|both>` |
| Ground-station container | `ros2 launch chassis_bringup ground_station.launch.py use_sim_time:=false view:=<drivetrain\|cloud\|openpose\|both>` |

- `camera:=cloud` adds the ZED point cloud, `camera:=openpose` OpenPose, `camera:=both` both. Use
  the ground station's `view:=` with the same name (`camera:=none` ↔ `view:=drivetrain`).
- No motors connected: add `can_interface:=mock` on the Jetson and skip `can-up`.
- On the ground-station **host**, open Foxglove Studio, connect to `ws://localhost:8765`, and
  import the layout for the camera mode ([docs/running.md → Foxglove](docs/running.md#foxglove)).
- To drive, hold **RB / R1** on the gamepad, then **RT** forward, **LT** reverse, **left stick** steers.

Every combination written out (sim or real, with or without ODESCs, each camera mode), the
simulator, a bench test, CAN testing and troubleshooting are in
**[docs/running.md](docs/running.md)**.

## Documentation

| Doc | Covers |
|---|---|
| [docs/setup.md](docs/setup.md) | `make setup <platform>` per machine (Mac, ARM64 Linux, x86, Jetson) and how it works |
| [docs/RUN_MODES.md](docs/RUN_MODES.md) | Every chassis / teleop / viewer mode as plain `ros2` commands, per platform |
| [docs/running.md](docs/running.md) | First-time setup, and every command per machine: sim, real drivetrain, camera modes, Foxglove, CAN, troubleshooting |
| [docs/architecture.md](docs/architecture.md) | How the drivetrain control stack fits together |
| [docs/people-pose.md](docs/people-pose.md) | ZED 2i point cloud, and OpenPose person and gesture detection (Jetson) |
| [docs/gazebo.md](docs/gazebo.md) | Gazebo rendering, GPU, and model notes |
| [docs/cheatsheet.md](docs/cheatsheet.md) | pixi and ROS 2 CLI commands |

Package docs: [chassis_bringup](ros2_ws/src/chassis_bringup/README.md) ·
[odesc](ros2_ws/src/odesc/README.md) · [teleop](ros2_ws/src/teleop/README.md) ·
[robot_description](ros2_ws/src/robot_description/README.md) ·
[zed2i](ros2_ws/src/perception/zed2i/README.md)

## Repository layout

```
.devcontainer/     dev container configs (x86 default, l4t/, mac/)
docker/            Dockerfiles and container entrypoint/setup scripts
tooling/           can-up (CAN bring-up, host), sim-up (one-command sim),
                   desktop-ros2 / rover-ros2 (run the images without VS Code),
                   billee-env.sh (platform + Pixi env lookup for make and the scripts)
devtool/           billee-dev: browser control panel for the repo's commands
Makefile           make setup/build/shell/sim
ros2_ws/pixi.toml  dependencies, environments, `build` task, ROS_DOMAIN_ID
ros2_ws/src/
  chassis_bringup/   launch files: sim, real, rover, ground station, viewers
  robot_description/ URDF/xacro, meshes, controllers.yaml
  odesc/             ros2_control hardware plugin for the ODESC motor controllers (SocketCAN)
  teleop/            gamepad → cmd_vel
  navigation/        EKF pose estimate + Nav2 controller/planner servers (sim)
  perception/        zed_openpose, people, people_msgs, zed2i (synthetic VIO)
  comms/, pxx/       in progress
  roboclaw_*         git submodules (upstream RoboClaw drivers)
```
