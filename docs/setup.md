# Machine setup with `make`

`make setup <platform>` is the scripted alternative to opening the dev container by hand
([README → 1. Open the dev container](../README.md#1-open-the-dev-container)): it installs the
host packages, builds the platform's image (or a native Pixi env on ARM64 Linux) and builds
the workspace, then `make build`, `make shell` and `make sim` follow the same platform. The
same commands are buttons in the browser control panel, `devtool/billee-dev`
([devtool/README.md](../devtool/README.md)).

## Quick start

```sh
git clone https://github.com/BILLEE-BroncoSTAR/billee-software-2027.git
cd billee-software-2027
make setup <mac | linux-aarch64 | l4t | x86>   # once per machine - see Setup below
make sim                                       # simulated rover + this machine's viewer
```

Then drive it (see [Run modes](RUN_MODES.md)). `make help` lists every command.

| Command | What it does |
|---|---|
| `make setup <platform>` | One-time setup of this machine: host packages, Pixi/Docker image, workspace build |
| `make build` | Rebuild `ros2_ws` after code changes |
| `make shell` | A shell with ROS 2 and the workspace sourced (`ros2 launch …` works as-is) |
| `make sim` | Gazebo sim + gamepad teleop + Foxglove bridge (:8765); opens RViz on Linux with a display |

All of them follow the platform you set up: natively on ARM64 Linux, through the
platform's Docker image everywhere else.

## Setup

Pick your machine. Each section is the whole process, top to bottom.

| Platform | Machine | Runs | Viewer |
|---|---|---|---|
| [`mac`](#apple-silicon-mac-mac) | Apple-Silicon Mac | Docker container (`mac-cpu` env) | Foxglove Studio |
| [`linux-aarch64`](#arm64-linux-linux-aarch64) | ARM64 Linux, no NVIDIA GPU (e.g. a Linux VM on a Mac) | natively (`linux-aarch64` env) | RViz (+ Foxglove) |
| [`x86`](#x86-64-linux--nvidia-x86) | x86-64 Linux + NVIDIA GPU (ground station) | Docker container (`default` env) | RViz (+ Foxglove) |
| [`l4t`](#nvidia-jetson-rover-l4t) | NVIDIA Jetson rover | Docker container (`l4t` env) | Foxglove / RViz on the ground station |

Need `make` first? `xcode-select --install` (Mac) or `sudo apt install -y make` (Linux).

### Apple-Silicon Mac (`mac`)

The Mac runs the simulation headless in a container and you view and drive it in
**Foxglove Studio** on the Mac.

**Needs:** Docker Desktop, [Foxglove Studio](https://foxglove.dev/download) (free
account), and optionally VS Code + the Dev Containers extension.

1. **Set up** (builds the `billee-mac-cpu` image with the workspace pre-built; the first
   run takes a while):
   ```sh
   make setup mac
   ```
2. **Start the sim.** Either from the Mac terminal:
   ```sh
   make sim
   ```
   or in VS Code: **Dev Containers: Reopen in Container → desktop-roshumble-mac-cpu**
   (reuses the image from step 1), then in a container terminal:
   ```sh
   ../tooling/sim-up
   ```
   Both run Gazebo headless (`xvfb-run`) with the `foxglove_bridge` on port **8765**,
   which is published to the Mac (`make sim`: Docker port mapping; VS Code: the
   devcontainer's `forwardPorts`). Wait for `diff_drive_controller` to report active.
3. **Connect Foxglove.** In Foxglove Studio: **Open connection → Foxglove WebSocket →**
   `ws://localhost:8765` → **Open**.
4. **Load the layout.** **Layouts → Import from file →**
   `ros2_ws/src/chassis_bringup/foxglove/drivetrain.json`. You get the grid, TF, the
   robot model (from `/robot_description`) and the odometry trail. If the robot shows
   as bare axes, see [running.md → Troubleshooting](running.md#troubleshooting).
5. **Drive.** Either
   - add a **Teleop** panel and set its topic to `/diff_drive_controller/cmd_vel_unstamped`, or
   - open a second terminal (`make shell` joins the running sim container, or open a new
     VS Code container terminal) and run
     ```sh
     ros2 run teleop_twist_keyboard teleop_twist_keyboard \
       --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
     ```

**Limits of the Mac container:** no USB gamepad, no `vcan0`, no RViz window, and no DDS
to other machines (Docker Desktop has no host networking). Other machines *can* open
the Mac's sim in Foxglove at `ws://<mac-ip>:8765` when it was started with `make sim`
(VS Code only forwards to `localhost`). To view the real rover, connect Foxglove to
`ws://<rover-ip>:8765` instead. For the fastest loop without Gazebo, use the mock
drivetrain: `ros2 launch chassis_bringup rover.launch.py mode:=real can_interface:=mock`.

### ARM64 Linux (`linux-aarch64`)

Runs natively (no Docker), so the gamepad, `vcan0` and RViz/Gazebo windows just work.
Everything renders on the CPU, so the sim is slower than on an NVIDIA machine.

**Needs:** a recent 64-bit ARM Linux with `curl` and `sudo`.

1. **Set up:** apt-installs `can-utils kmod xvfb` (only what's missing), installs Pixi
   to `~/.pixi`, then installs and builds the `linux-aarch64` environment in `ros2_ws/.pixi`:
   ```sh
   make setup linux-aarch64
   ```
2. **Run:** `make sim` opens Gazebo and **RViz** and also serves Foxglove on :8765.
   Plug in a gamepad to drive (hold RB; RT forward, LT reverse, left stick steers). For the ODESC
   shadow, run `tooling/can-up vcan0` first and `make sim` picks it up.
3. **Without make:** `cd ros2_ws && pixi shell -e linux-aarch64` gives a ROS 2 terminal for
   every command in [docs/RUN_MODES.md](RUN_MODES.md).
4. **VS Code Python:** interpreter `ros2_ws/.pixi/envs/linux-aarch64/bin/python3`.

In a VM, use bridged networking if it should talk DDS to the rover or another machine.

### x86-64 Linux + NVIDIA (`x86`)

The ground-station setup: GPU-accelerated Gazebo, RViz, the gamepad and F' GDS.

**Needs:** Ubuntu 24.04 or newer, an NVIDIA GPU, `sudo`, and `make`
(`sudo apt install -y make`). Everything else is installed for you. Optionally VS Code +
Remote Development extensions.

1. **Set up:**
   ```sh
   make setup x86
   ```
   First, `tooling/install-x86-host` prepares the machine. Each step is skipped when it's
   already done:

   | Step | Installs |
   |---|---|
   | Base packages | `curl`, `gnupg`, `make`, `can-utils`, `kmod`, `xhost`, `ubuntu-drivers-common` |
   | NVIDIA driver | Ubuntu's recommended driver (`ubuntu-drivers install`), only if the current one doesn't support the image's CUDA version (13.2, read from `docker/Dockerfile.desktop.humble`) |
   | Docker Engine | from Docker's apt repository, and adds you to the `docker` group |
   | NVIDIA Container Toolkit | from NVIDIA's apt repository, registered as Docker's `nvidia` runtime |
   | Smoke test | `nvidia-smi` inside a container with `--gpus all` |

   A new driver needs a **reboot** and a new group needs a **re-login**: setup stops and
   tells you which. Do it, then run `make setup x86` again; it continues from where it
   stopped. Then it builds `billee-desktop:latest` (CUDA + ZED SDK) and the `default` env
   inside it.

   - Preview without changing anything: `DRY_RUN=1 tooling/install-x86-host`
   - Pick a driver branch instead of Ubuntu's recommendation:
     `NVIDIA_DRIVER_BRANCH=<n> make setup x86` (runs `ubuntu-drivers install nvidia:<n>`)
   - With **Secure Boot** on, the driver install asks for a password, and on the next boot
     you choose **Enroll MOK** and enter it.
   - Already set up by hand, or not on Ubuntu: `make setup x86 SKIP_APT=1` skips this step
     and only checks that Docker and the `nvidia` runtime work.
2. **Run:** `make sim` (Gazebo + **RViz**), or `make shell` and e.g.
   `ros2 launch chassis_bringup ground_station.launch.py` to operate the real rover.
3. **VS Code (optional):** **Reopen in Container → desktop-roshumble_dev-x862** shares
   the same image and environment. Python interpreter `ros2_ws/.pixi/envs/default/bin/python3`.

### NVIDIA Jetson rover (`l4t`)

The rover itself: a Jetson (Orin-family or Thor) on **JetPack 6.2 / L4T r36.4**.

**Needs:** the one-time Docker + `nvidia` runtime host prep in
[README → 1. Open the dev container](../README.md#1-open-the-dev-container) (redo it after a reflash).

1. **Set up:** checks the Docker `nvidia` runtime, installs `can-utils kmod`, builds
   `rover-ros2:latest`, then the `l4t` env inside it (first build uses 2 workers,
   sequential, to fit in RAM):
   ```sh
   make setup l4t
   ```
2. **Run:** `tooling/can-up` (brings `can2` up), then `make shell` and
   `ros2 launch chassis_bringup rover.launch.py mode:=real`. The rover is headless:
   view from the ground station with Foxglove (`ws://<rover-ip>:8765`) or RViz over DDS.
3. **VS Code (optional):** **Reopen in Container → rover-roshumble_l4t-aarch64**;
   interpreter `ros2_ws/.pixi/envs/l4t/bin/python3`.


### How setup works

- `make setup <platform>` records the platform in `ros2_ws/.billee-platform`
  (gitignored). `make build/shell/sim`, `tooling/sim-up`, `tooling/desktop-ros2` and the
  container shell config all read it through `tooling/billee-env.sh`, so nothing has to be
  edited per machine. Without the file they detect it (x86-64 → `x86`, Jetson → `l4t`,
  other ARM64 → `linux-aarch64`); `BILLEE_PLATFORM=<platform>` overrides one command.
- Each platform has its own Pixi environment, named after it except where noted:

  | Platform | Pixi environment | Packages |
  |---|---|---|
  | `mac` | `mac-cpu` | linux-aarch64, CPU-only (runs in the Mac's Linux container) |
  | `linux-aarch64` | `linux-aarch64` | same set as `mac-cpu`, locked to identical versions (shared solve-group) |
  | `l4t` | `l4t` | same package list, locked separately so the rover only changes when tested on it |
  | `x86` | `default` | linux-64 + CUDA 13 |

  Switching a checkout to another environment is safe: `make setup`/`make build` notice a
  build made with a different environment and rebuild from scratch.
- Containers mount the workspace at the same path as their VS Code devcontainer, so the
  Pixi env and `install/` work from both. `tooling/desktop-ros2 build|run|shell` is the
  no-VS-Code way in (`tooling/rover-ros2` = the same, pinned to `l4t`).
- One checkout = one platform. The Pixi env and colcon `install/` embed absolute paths,
  so switching a checkout between a native build and a container means a reinstall.
- `SKIP_APT=1` skips the apt step. More on Pixi: [cheat sheet](cheatsheet.md),
  [Pixi in VS Code](https://pixi.prefix.dev/latest/integration/editor/vscode/#python-extension).

