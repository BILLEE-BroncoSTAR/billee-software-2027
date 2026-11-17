# Dev containers and images (`.devcontainer/`, `docker/`)

Everything runs in a container. ROS 2 Humble, colcon and every ROS dependency come from pixi
(RoboStack, [`ros2_ws/pixi.toml`](../ros2_ws/pixi.toml)), not from the image. The images add only
what pixi can't provide: CUDA, the ZED SDK, OpenPose, GPU rendering and the camera tools. How to
open each container is in [README §1](../README.md#1-open-the-dev-container).

| Dev container | Config | Image | Pixi env |
|---|---|---|---|
| `desktop-roshumble_dev-x862` (x86 + NVIDIA) | `.devcontainer/devcontainer.json` | `docker/Dockerfile.desktop.humble` | `default` |
| `rover-roshumble_l4t-aarch64` (Jetson) | `.devcontainer/l4t/devcontainer.json` | `docker/Dockerfile.l4t-humble` | `l4t` |
| `desktop-roshumble-mac-cpu` (Apple Silicon) | `.devcontainer/mac/devcontainer.json` | `docker/Dockerfile.desktop.mac.humble` | `mac-cpu` |

`tooling/desktop-ros2` and `tooling/rover-ros2` build and run the first two without VS Code
([tooling.md](tooling.md#desktop-ros2-rover-ros2)).

## Pixi (`ros2_ws/pixi.toml`)

- Environments: `default` (linux-64 with CUDA 13), `l4t` and `mac-cpu` (both linux-aarch64, CPU
  packages; the Jetson's CUDA comes from the image, not pixi).
- `pixi run build` is a `colcon build --symlink-install` over `src/` in Release. The x86 variant
  also links against the CUDA stubs.
- `pixi run fmt` runs `ruff check src/ --fix`.
- Activation sets `ROS_DOMAIN_ID=42` and sources `install/local_setup.sh`, the workspace overlay
  only (not `setup.sh`, which can replay an underlay built on another architecture).
- `pixi.lock` pins every package. `pixi add` / `pixi remove` re-solve it.

## Jetson image (`Dockerfile.l4t-humble`)

Base: NVIDIA's Isaac ROS Humble image (L4T 36.4 / JetPack 6.x: CUDA 12.6, cuDNN 9, TensorRT 10.3,
OpenCV 4.5.4). In order, it adds:

1. `ros_user` with passwordless sudo (the launch files use `sudo -n` to drop the page cache).
2. The ZED SDK 5.4 for L4T 36.4, made world-readable (VS Code remaps `ros_user`'s uid).
3. OpenPose, built from source by `docker/build_openpose.sh` into `/opt/openpose`. `OPENPOSE_JOBS`
   sets the parallel jobs (3 in the dev container: about 21 min on the 8 GB Orin Nano). The
   patches it applies are listed in [people-pose.md](people-pose.md#the-jetson-image).
4. `gpu_groups.sh` and `zed_udev.sh` into `/usr/local/bin` (run at container start, below).
5. Xvfb and VirtualGL 3.1.5 (checksum-pinned), for the headless Gazebo sim
   ([gazebo.md](gazebo.md#headless-gpu-rendering-jetson)).
6. `gstreamer1.0-tools` and `gstreamer1.0-x`, for the CSI cameras ([cameras.md](cameras.md)).
7. pixi in `/opt/pixi`.

The OpenPose layers come early so that changing anything after them doesn't recompile OpenPose.

### l4t dev container settings

- `--runtime nvidia` (not `--gpus all`), privileged, host network, PID and IPC namespaces,
  hostname `rover`.
- `--group-add` video (44), 104 and 995: the GPU device groups, given to every process including
  VS Code's terminals. 104 and 995 are this Jetson's gids; after a reflash check them with
  `stat -c '%G %g %n' /dev/nvmap /dev/dri/renderD128 /dev/nvhost-dbg-gpu` on the host.
- Mounts: `/dev`, `/dev/shm`, the host's `/etc/udev/rules.d` and `/run/udev` (for the ZED rule),
  `/tmp/.X11-unix`, and the host's `/tmp` at `/host/tmp` (for the Argus socket).
- `postStartCommand`, at every container start:
  1. links `/tmp/argus_socket` to the host's (`/host/tmp/argus_socket`). Linking rather than
     mounting the socket survives `systemctl restart nvargus-daemon`;
  2. `gpu_groups.sh`: writes the host's GPU device gids into the container's `/etc/group` and adds
     `ros_user`, for shells opened later with `docker exec`;
  3. `zed_udev.sh`: installs the ZED SDK's `99-slabs.rules` on the **host** and reloads udev.
     Without it the ZED's sensor device is root-only (`[ZED][MCU] Permissions denied`);
  4. copies `configs/zed/SN*.conf` into `/usr/local/zed/settings/` (below);
  5. `pixi install --environment l4t`.

### ZED calibration (`configs/zed/`)

`SN<serial>.conf` is a ZED camera's factory calibration; `SN36243153.conf` is the team's ZED 2i.
Copying it in at start lets the camera open without internet; without it the SDK downloads it
from Stereolabs on the first open, into the container, and a rebuild loses it.

To add a camera: open it once in the Jetson container with internet (e.g.
`ros2 launch people people_rover.launch.py openpose:=false`), then copy
`/usr/local/zed/settings/SN<its serial>.conf` into `configs/zed/` and commit it.

## x86 image (`Dockerfile.desktop.humble`)

Base: `nvidia/cuda` 13.2 devel on Ubuntu 22.04. Adds the ZED SDK 5.4 (CUDA 13 build), the
`ppa:kisak/kisak-mesa` Mesa (22.04's is too old for newer GPUs), `ros_user` with passwordless sudo,
pixi, and `docker/terminal_config.sh` appended to `~/.bashrc`. No OpenPose: the people node is
Jetson-only.

The dev container runs `xhost +local:` on the host first, passes `--gpus all` and the X socket,
allows USB (189) and video4linux (81) devices (`--device-cgroup-rule`), and runs
`pixi install` as its `updateContentCommand`. At container start it also uses the same
`zed_udev.sh` helper and host udev mounts as the L4T profile, so the ZED 2's HID sensor device
has the SDK's `99-slabs.rules` permissions. A Docker named volume mounted at
`/usr/local/zed/resources` preserves the ZED SDK's downloaded NEURAL_LIGHT and body-tracking
models (including `skeleton_body18_3.2`) and their optimized engines across rebuilds; Docker
creates the volume automatically.

## Mac image (`Dockerfile.desktop.mac.humble`)

Plain Ubuntu 22.04 with pixi, `mac-cpu` installed and the workspace built into the image. The
container runs as root with `ros2_ws` mounted at `/root/ros2_ws`, forwards port 8765 for Foxglove
on macOS (Docker Desktop has no host networking), and forces Mesa's `llvmpipe` software renderer.
Its `postStartCommand` reinstalls the pixi env when it was built for another path.

## Shell setup (`docker/terminal_config.sh`)

Appended to `~/.bashrc` in the x86 and Mac images. It finds the workspace, sources the active
pixi env's `setup.bash` and the workspace's `install/local_setup.bash`, `cd`s into `ros2_ws`, and
prints which pixi env is active (or how to enter one).

## Not used by the dev containers

`docker/compose.yaml` (a Compose service for the Jetson image) and `docker/ros_entrypoint.sh` /
`ros_entrypoint_jetson.sh` (welcome banners from the Stereolabs images) aren't referenced by any
dev container, Dockerfile or script. The entrypoints export `ROS_DOMAIN_ID=0`, not the rover's 42.

The x86 and Jetson dev containers disable VS Code's automatic port forwarding (so does
`.vscode/settings.json`): with host networking and the host's PID namespace, its port scanner sees
its own forwarding helper and spawns forwards without end.
