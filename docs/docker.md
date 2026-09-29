# Containers

Every BILLEE machine runs the ROS 2 stack in a container. Each platform's container is one
service in [`docker/compose.yaml`](../docker/compose.yaml), which is the **only** place
its image, GPU access, devices, mounts and environment are defined. `make`,
`tooling/desktop-ros2` and the VS Code devcontainers all start that same service.

| Platform (`make setup …`) | Service | Image | Dockerfile | Devcontainer |
|---|---|---|---|---|
| `x86` — x86 Linux + NVIDIA | `x86` | `billee-desktop:latest` | `Dockerfile.desktop.humble` | `.devcontainer/x86` |
| `wsl` — Windows + WSL2 + NVIDIA | `wsl` | `billee-desktop:latest` | `Dockerfile.desktop.humble` | `.devcontainer/wsl` |
| `mac` — Apple-Silicon Mac | `mac` | `billee-mac-cpu:latest` | `Dockerfile.desktop.mac.humble` | `.devcontainer/mac` |
| `l4t` — Jetson rover (JetPack 6) | `rover` | `rover-ros2:latest` | `Dockerfile.l4t-humble` | `.devcontainer/l4t` |

Every service mounts the repo at `/workspaces/billee-software-2027` and starts in its
`ros2_ws`. The Pixi environment and colcon's `install/` embed absolute paths, so this one
fixed path is what lets a workspace built from `make` work in VS Code and the other way
around. `mac` and `rover` are pinned to `platform: linux/arm64`, so they build as ARM64
anywhere (on an x86 machine, slowly, through emulation).

## Everyday commands

From the repo root on the host (the platform comes from `make setup`):

```sh
make up        # start the container in the background (no-op if it is running)
make shell     # ROS 2 terminal in it (starts it first if needed); run again for more
make sim       # or: make ground / make rover / make build
make down      # stop it
```

Plain compose works too, with any service name from the table:

```sh
docker compose -f docker/compose.yaml build x86
docker compose -f docker/compose.yaml up -d x86
docker compose -f docker/compose.yaml exec x86 bash
docker compose -f docker/compose.yaml stop x86
```

Services run `sleep infinity`; shells and launches exec into the running container.
`tooling/container-init` installs the Pixi environment on start (the devcontainers'
`postStartCommand`, and before every `tooling/desktop-ros2 run`). It is a no-op when the
environment is current.

## Changing a setting

Edit `docker/compose.yaml` and restart the container (`make down && make up`, or
**Rebuild Container** in VS Code). The shared pieces are YAML anchors at the top:

- `x-build-args` — `PIXI_VERSION`, passed to every image (the only copy of it)
- `x-workspace` — the repo mount path, working dir, `init`, `sleep infinity`
- `x-linux-host` — privileged, host network / PID / IPC (Linux hosts: DDS, `can0`/`vcan0`)
- `x-gpu` — the NVIDIA GPU reservation (x86, wsl)

The Jetson's ZED SDK build is picked by `L4T_MAJOR`/`L4T_MINOR` under the `rover` service.
They must match the Jetson's `cat /etc/nv_tegra_release`; the build fails if Stereolabs
has no ZED SDK for that release.

## VS Code

**Dev Containers: Reopen in Container** and pick the platform's entry. It starts the same
compose service, so a container already started by `make up` is reused. If VS Code
recreates it (for example after **Rebuild Container**), `make shell` simply joins the new
one: `tooling/desktop-ros2` never restarts a running service.

## What stays on the host

Only what can't live in a container:

| Platform | Host needs |
|---|---|
| `x86` | NVIDIA driver, Docker Engine, NVIDIA Container Toolkit, `make`, `xhost`; `can-utils`/`kmod` for `tooling/can-up` (all installed by `make setup x86`) |
| `wsl` | NVIDIA driver on Windows, Docker Desktop with WSL integration, `make` |
| `mac` | Docker Desktop, `make`, Foxglove Studio |
| `l4t` | JetPack 6, Docker + nvidia runtime, `can-utils`/`kmod`, `tooling/can0.service` |

## Notes

- tmpfs mounts create directories in the host machine's RAM (volatile): anything written
  there is gone when the container stops.
