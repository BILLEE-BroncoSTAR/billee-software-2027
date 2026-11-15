# BILLEE rover software (URC 2027)

ROS 2 Humble workspace for the BILLEE rover: a six-wheel differential drive on ODESC motor
controllers over CAN, a Gazebo simulation of it, gamepad teleop, and ZED 2i perception.
Dependencies are managed by [pixi](https://pixi.sh) and everything runs in a dev container.

## Start here (new machine)

```bash
git clone https://github.com/BILLEE-BroncoSTAR/billee-software-2027.git
cd billee-software-2027
./starthere
```

`./starthere` opens the **control panel** in your browser (<http://localhost:7700>), where every
sim, drivetrain, viewer and check is a button, and sets this machine up if it isn't yet: it works
out what kind of machine this is, installs what's missing, fetches the git submodules and builds
the workspace. Setup runs in the terminal, so answer its **sudo password prompt** there; the panel
shows its progress and says when the machine is ready. Leave the terminal open while you use the
panel (Ctrl-C stops it). Later, `./starthere` (or `devtool/billee-dev`) just opens the panel.
`./starthere --preview` shows what setup would install without changing anything.

| Your machine | What setup does | Before you start |
|---|---|---|
| x86-64 Linux, no NVIDIA GPU (most laptops) | apt packages, Pixi, the ROS 2 environment, native build. No Docker. | `git`, `curl`, `python3` (`./starthere` tells you if one is missing) |
| x86-64 Linux + NVIDIA GPU (ground station) | NVIDIA driver, Docker, NVIDIA Container Toolkit, the desktop image, build | If it asks for a reboot or a new login, do that and run `./starthere` again |
| Apple-Silicon Mac | The `mac-cpu` image with the workspace built in | Docker Desktop running; `xcode-select --install` for `git`, `make`, `python3` |
| NVIDIA Jetson (the rover) | The rover image (pulls a ~12.6 GB base, compiles OpenPose in ~21 min), build | Docker with the `nvidia` runtime: the Jetson host steps in [1. Open the dev container](#1-open-the-dev-container) |
| Other ARM64 Linux | Pixi and a native build | `git`, `curl`, `python3` |

The first setup downloads the ROS 2 environment (about 10 GB on disk) and takes 10–30 minutes
on a normal connection, longer on the Jetson; later runs start in seconds.

**First drive, in the control panel:**

1. **Drivetrain → Gazebo simulation → Start** (or **Mock drivetrain** for the real control stack
   with no Gazebo and no motors).
2. **External → Foxglove Studio**: connect to `ws://localhost:8765` and import
   `ros2_ws/src/chassis_bringup/foxglove/drivetrain.json`.
3. **Drive it → Scripted drive**, **Keyboard teleop**, or **Gamepad teleop** (hold LB, then RT
   forward, LT reverse, left stick steers).

**Setup doesn't cover these, do them once by hand:**

- Install the [Foxglove Studio](https://foxglove.dev/download) desktop app on the machine you view from.
- On the Jetson: **Perception & pose → Download OpenPose models**, and for the real motors
  **CAN bus → Bring up a real CAN bus** (or **Persist can2 across reboots**).
- A ZED 2i on a machine without an NVIDIA GPU: plug it into a USB 3 port, then
  **Perception & pose → ZED over USB**.

Prefer the terminal or VS Code? Everything above is also plain commands: `make setup <platform>`
([docs/setup.md](docs/setup.md)), or the dev containers below.

## 1. Open the dev container

The manual route, e.g. for VS Code. Pick the configuration for your machine
(VS Code: **Dev Containers: Reopen in Container**):

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
[docs/setup.md](docs/setup.md). `./starthere` ([Start here](#start-here-new-machine)) does the same through the
control panel ([devtool/README.md](devtool/README.md)).

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
- No motors connected: use the virtual bus instead ([docs/running.md → CAN testing without motors](docs/running.md#can-testing-without-motors)),
  or `can_interface:=mock` for no CAN at all (the driver loops the commanded speed back).
- On the ground-station **host**, open Foxglove Studio, connect to `ws://localhost:8765`, and
  import the layout for the camera mode ([docs/running.md → Foxglove](docs/running.md#foxglove)).
- To drive, hold **LB / L1** on the gamepad, then **RT** forward, **LT** reverse, **left stick**
  steers (arcade, the default). For the two-stick tank drive, add `drive_scheme:=tank` to the
  ground-station launch ([docs/teleop.md](docs/teleop.md)).

Every combination written out (sim or real, with or without ODESCs, each camera mode), the
simulator, a bench test, CAN testing and troubleshooting are in
**[docs/running.md](docs/running.md)**.

## Documentation

Everything the rover runs is documented in [`docs/`](docs):

| Doc | Covers |
|---|---|
| [setup.md](docs/setup.md) | `make setup <platform>` per machine (Mac, ARM64 Linux, x86, Jetson) and how it works; `./starthere` runs it |
| [running.md](docs/running.md) | First-time setup, and every command per machine: sim, real drivetrain, camera modes, Foxglove, CAN, troubleshooting |
| [RUN_MODES.md](docs/RUN_MODES.md) | Every chassis / teleop / viewer mode as plain `ros2` commands, per platform |
| [architecture.md](docs/architecture.md) | How the drivetrain control stack, odometry and TF fit together |
| [launch.md](docs/launch.md) | `chassis_bringup`: every launch file, what it starts, its arguments and configs |
| [odesc.md](docs/odesc.md) | `odesc`: the ODESC `ros2_control` driver, CAN protocol, wheel health, vCAN emulator, mock mode, board configs (`configs/odesc/`) |
| [robot-model.md](docs/robot-model.md) | `robot_description`: URDF/xacro, meshes, `controllers.yaml` |
| [teleop.md](docs/teleop.md) | `teleop`: gamepad arcade or tank drive |
| [navigation.md](docs/navigation.md) | `navigation`: the EKFs and the Nav2 controller/planner servers |
| [people-pose.md](docs/people-pose.md) | `zed_openpose`, `people`, `people_msgs`: ZED 2i point cloud, VIO, OpenPose and gestures (Jetson) |
| [zed_usb](ros2_ws/src/perception/zed_usb/README.md) | `zed_usb`: the ZED 2i point cloud on a machine without an NVIDIA GPU (USB video + CPU stereo) |
| [cameras.md](docs/cameras.md) | `camera_stream`: the five CSI cameras; vendored `foxglove_msgs` |
| [synthetic-vio.md](docs/synthetic-vio.md) | `zed2i`: synthetic VIO for the sim's EKF |
| [gazebo.md](docs/gazebo.md) | Gazebo rendering, GPU, and model notes |
| [containers.md](docs/containers.md) | Dev containers, Docker images, pixi environments, ZED calibration (`configs/zed/`) |
| [tooling.md](docs/tooling.md) | `tooling/`: `can-up`, `can2.service`, `can-clear-faults`, `sim-up`, `zed-usb`, `desktop-ros2` / `rover-ros2` |
| [devtool/README.md](devtool/README.md) | The control panel (`./starthere`, `devtool/billee-dev`) |
| [cheatsheet.md](docs/cheatsheet.md) | pixi and ROS 2 CLI commands |

## Repository layout

```
starthere          first run on a new machine: control panel + machine setup     README.md
devtool/           billee-dev: browser control panel for the repo's commands    devtool/README.md
Makefile           make setup/build/shell/sim                                   docs/setup.md
.devcontainer/     dev container configs (x86 default, cpu/, l4t/, mac/)        docs/containers.md
docker/            Dockerfiles and container setup scripts                      docs/containers.md
configs/odesc/     ODESC board configs (odrivetool backups)                     docs/odesc.md
configs/zed/       ZED factory calibration                                      docs/containers.md
tooling/           CAN bring-up and fault clearing, sim-up, zed-usb, runners    docs/tooling.md
ros2_ws/pixi.toml  dependencies, environments, `build` task, ROS_DOMAIN_ID
ros2_ws/src/
  chassis_bringup/   launch files: sim, real, rover, ground station, viewers   docs/launch.md
  robot_description/ URDF/xacro, meshes, controllers.yaml                      docs/robot-model.md
  odesc/             ros2_control plugin for the ODESC motor controllers       docs/odesc.md
  teleop/            gamepad → cmd_vel                                         docs/teleop.md
  navigation/        EKF pose estimate + Nav2 controller/planner servers       docs/navigation.md
  camera_stream/     CSI cameras → H.264 + JPEG                                docs/cameras.md
  perception/        zed_openpose, people, people_msgs                         docs/people-pose.md
                     zed2i (synthetic VIO)                                     docs/synthetic-vio.md
  third_party/       foxglove_msgs, vendored (VENDORED.md)                     docs/cameras.md
```

Not part of the running rover (no launch file starts them): `ros2_ws/src/comms/` (a GPS node, in
progress), `ros2_ws/src/pxx/` (a motor hardware plugin, in progress), and the `roboclaw_*` git submodules (upstream
RoboClaw drivers, unused by the ODESC drivetrain).
