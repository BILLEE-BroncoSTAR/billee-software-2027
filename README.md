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
| Jetson host (not the container), once per boot | `cd ~/billee-software-2027 && tooling/can-up` |
| Jetson container | `ros2 launch chassis_bringup rover.launch.py mode:=real camera:=<none\|cloud\|openpose\|both>` |
| Ground-station container | `ros2 launch chassis_bringup ground_station.launch.py use_sim_time:=false view:=<drivetrain\|cloud\|openpose\|both>` |

- `camera:=cloud` adds the ZED point cloud, `camera:=openpose` OpenPose, `camera:=both` both. Use
  the ground station's `view:=` with the same name (`camera:=none` ↔ `view:=drivetrain`).
- No motors connected: use the virtual bus instead ([docs/running.md → CAN testing without motors](docs/running.md#can-testing-without-motors)).
- On the ground-station **host**, open Foxglove Studio, connect to `ws://localhost:8765`, and
  import the layout for the camera mode ([docs/running.md → Foxglove](docs/running.md#foxglove)).
- To drive, hold **LB / L1** on the gamepad and move the two sticks up and down.

Every combination written out (sim or real, with or without ODESCs, each camera mode), the
simulator, a bench test, CAN testing and troubleshooting are in
**[docs/running.md](docs/running.md)**.

## Documentation

Everything the rover runs is documented in [`docs/`](docs):

| Doc | Covers |
|---|---|
| [running.md](docs/running.md) | First-time setup, and every command per machine: sim, real drivetrain, camera modes, Foxglove, CAN, troubleshooting |
| [architecture.md](docs/architecture.md) | How the drivetrain control stack, odometry and TF fit together |
| [launch.md](docs/launch.md) | `chassis_bringup`: every launch file, what it starts, its arguments and configs |
| [odesc.md](docs/odesc.md) | `odesc`: the ODESC `ros2_control` driver, CAN protocol, wheel health, vCAN emulator, board configs (`configs/odesc/`) |
| [robot-model.md](docs/robot-model.md) | `robot_description`: URDF/xacro, meshes, `controllers.yaml` |
| [teleop.md](docs/teleop.md) | `teleop`: gamepad tank drive |
| [navigation.md](docs/navigation.md) | `navigation`: the EKFs and the Nav2 controller/planner servers |
| [people-pose.md](docs/people-pose.md) | `zed_openpose`, `people`, `people_msgs`: ZED 2i point cloud, VIO, OpenPose and gestures (Jetson) |
| [cameras.md](docs/cameras.md) | `camera_stream`: the five CSI cameras; vendored `foxglove_msgs` |
| [arm-camera.md](docs/arm-camera.md) | `arm_vision`, `arm_vision_msgs`: the arm's RealSense D435i (driver, SDK, udev), the RGB-D CV pipeline and ArUco detection (Jetson) |
| [synthetic-vio.md](docs/synthetic-vio.md) | `zed2i`: synthetic VIO for the sim's EKF |
| [gazebo.md](docs/gazebo.md) | Gazebo rendering, GPU, and model notes |
| [containers.md](docs/containers.md) | Dev containers, Docker images, pixi environments, ZED calibration (`configs/zed/`) |
| [tooling.md](docs/tooling.md) | `tooling/`: `can-up`, `can2.service`, `argus-watchdog` (restarts a wedged camera daemon), `can-clear-faults`, `sim-up`, `desktop-ros2` / `rover-ros2` |
| [cheatsheet.md](docs/cheatsheet.md) | pixi and ROS 2 CLI commands |

## Repository layout

```
.devcontainer/     dev container configs (x86 default, l4t/, mac/)          docs/containers.md
docker/            Dockerfiles and container setup scripts                  docs/containers.md
configs/odesc/     ODESC board configs (odrivetool backups)                 docs/odesc.md
configs/zed/       ZED factory calibration                                  docs/containers.md
tooling/           CAN bring-up and fault clearing, Argus watchdog, sim-up, image runners   docs/tooling.md
ros2_ws/pixi.toml  dependencies, environments, `build` task, ROS_DOMAIN_ID
ros2_ws/src/
  chassis_bringup/   launch files: sim, real, rover, ground station, viewers   docs/launch.md
  robot_description/ URDF/xacro, meshes, controllers.yaml                      docs/robot-model.md
  odesc/             ros2_control plugin for the ODESC motor controllers       docs/odesc.md
  teleop/            gamepad → cmd_vel                                         docs/teleop.md
  navigation/        EKF pose estimate + Nav2 controller/planner servers       docs/navigation.md
  camera_stream/     CSI cameras → H.264 + JPEG                                docs/cameras.md
  perception/        zed_openpose, people, people_msgs                         docs/people-pose.md
                     arm_vision, arm_vision_msgs (arm RealSense + ArUco)       docs/arm-camera.md
                     zed2i (synthetic VIO)                                     docs/synthetic-vio.md
  third_party/       foxglove_msgs, vendored (VENDORED.md)                     docs/cameras.md
```

Not part of the running rover (no launch file starts them): `ros2_ws/src/comms/` (a GPS node, in
progress), `ros2_ws/src/pxx/` (a motor hardware plugin, in progress), and the `roboclaw_*` git submodules (upstream
RoboClaw drivers, unused by the ODESC drivetrain).
