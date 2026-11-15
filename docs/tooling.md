# Tooling scripts (`tooling/`)

Scripts run from the repository root. Only the CAN ones touch the rover's hardware.

| Script | Runs on | Does |
|---|---|---|
| [`can-up`](#can-up) | Jetson **host** | brings a CAN interface up or down |
| [`can2.service`](#can2service) | Jetson **host** (systemd) | brings `can2` up at every boot |
| [`can-clear-faults`](#can-clear-faults) | host or container | clears latched ODESC errors, optionally reboots a board |
| [`sim-up`](#sim-up) | container | one-command Gazebo sim |
| [`desktop-ros2`, `rover-ros2`](#desktop-ros2-rover-ros2) | host | build and run the dev images without VS Code |
| [`zed-usb`](../ros2_ws/src/perception/zed_usb/README.md) | host | the ZED 2i point cloud on a machine without an NVIDIA GPU |
| `host-ros2` | x86 host, no NVIDIA GPU | run a command in the `default` Pixi env natively, no Docker |

## can-up

```bash
tooling/can-up                  # can2 @ 500 kbit/s, restart-ms 100, txqueuelen 100
tooling/can-up can3             # another interface (the USB-CAN adapter's number can change)
tooling/can-up can2 250000      # another bitrate
tooling/can-up vcan0            # create and bring up a virtual bus
tooling/can-up status [iface]   # show the state (default can2)
tooling/can-up down [iface]     # take it down; a vcanN is also deleted
```

- **Execute it, don't `source` it.** Sourced, its `set -e` and `exit` would close your shell (and
  the container, if that shell is its main process); it refuses and says so.
- **Host only.** The container has no `ip` (iproute2); the script stops with an error there.
  Every `ip` call uses `sudo`.
- A real `canN` must already exist in the kernel (Tegra mttcan or a plugged-in USB-CAN adapter);
  the script configures it but can't create it. A `vcanN` is created on demand (`modprobe vcan`).
- `txqueuelen 100` is set even on an interface that is already UP. The kernel default of 10 is too
  small for the ODESC driver's 12-frame burst per control cycle and gives `ENOBUFS`
  ([odesc.md](odesc.md#can-command-subset-used)). Changing the bitrate needs `down` first.
- Idempotent: running it again on an UP interface only reports the state.

## can2.service

A systemd unit that does what `tooling/can-up` does for `can2`, at boot:

```bash
sudo cp tooling/can2.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now can2.service
systemctl status can2.service
```

It is bound to the `can2` device, so it does nothing if the adapter enumerates under another
number. The bitrate (500000) is in its `ExecStart` line.

## can-clear-faults

Python standard library only, so it runs on the host or in the container. The interface must be
up already.

```bash
tooling/can-clear-faults                     # every node in odesc/config/node_map.yaml, on can2
tooling/can-clear-faults --nodes 5           # one node (or a comma-separated list)
tooling/can-clear-faults --status            # only report; send nothing
tooling/can-clear-faults --nodes 5 --reboot  # also reboot a board a clear can't fix
tooling/can-clear-faults --iface vcan0
```

For each node it listens to the heartbeat (`0x01`), sends `Clear_Errors` (`0x18`), and listens
again, printing the axis state and the error names (from the fw 0.5.1 interface definition).
`Clear_Errors` clears error bits only; it doesn't arm or move a motor.

`BrakeResistorDisarmed` (`0x10`) can't be cleared on this firmware: the brake resistor re-arms only
at boot. With `--reboot`, such a board gets CANSimple `Reset` (`0x16`), the same reboot `odrivetool`
does, and the script waits up to 20 s for it to come back and clears again. **The rebooted board
runs its startup sequence**, an encoder offset calibration that turns the motor and then closed
loop, so keep the wheels off the ground.

Exit status: `0` every listed node answered with no error, `1` an error remains or a node didn't
answer, `2` usage or interface problem.

## sim-up

```bash
tooling/sim-up [foxglove] [joystick] [can_sim]     # each true|false, defaults true true false
tooling/sim-up true false                           # no gamepad
tooling/sim-up true true true                       # + ODESC driver on vcan0 (can-up vcan0 first)
```

Runs `sim_gz.launch.py` with `foxglove`, `joy_control` and `odesc_shadow` set from its arguments.
It uses the active pixi env (`$PIXI_ENVIRONMENT_NAME`), or picks one from the machine (x86 →
`default`, Jetson → `l4t`, else `mac-cpu`). Without a usable display (none, or a stale `$DISPLAY`
that `xdpyinfo` can't reach), it sets `gui:=false` and wraps the launch in `xvfb-run -a vglrun -d
egl` ([gazebo.md](gazebo.md#headless-gpu-rendering-jetson)).

It starts teleop, so don't use it on the Jetson while the ground station also runs teleop: every
command would be published twice. Use `rover.launch.py` there.

## desktop-ros2, rover-ros2

Build and run the dev images without VS Code, with the same devices, networking and mounts as the
dev containers ([containers.md](containers.md)):

```bash
tooling/desktop-ros2 build|run|shell [args...]   # platform from make setup (x86, mac, l4t)
tooling/rover-ros2   build|run|shell [args...]   # = BILLEE_PLATFORM=l4t tooling/desktop-ros2
```

- `build` passes extra arguments to `docker build` (e.g. `--build-arg OPENPOSE_JOBS=3`).
- `run` runs the extra arguments (default: `pixi shell` in the platform's env) in
  `billee-<platform>`, starting it if it isn't running and joining it if it is, so a second
  command (e.g. keyboard teleop next to the drivetrain) shares one container.
- `shell` opens Bash in it.

On x86 and the Jetson the whole repository is mounted at `/workspaces/<repo>` (working directory
`ros2_ws`), with host networking, `/dev` and `--privileged`. Every container start runs
`pixi install` for the platform's env. On the Jetson (`rover-ros2`, `--runtime nvidia`, as root,
so no GPU-group setup is needed) the start also links the host's Argus socket for the CSI cameras
and copies the ZED's factory calibration from `configs/zed/`, as the l4t dev container's
`postStartCommand` does. `./starthere` / `make setup l4t` and the control panel run everything on
the Jetson through it.

Files the container writes in the repository (`ros2_ws/build`, `install`, `.pixi`) are owned by
root on the host. Don't build the same clone from both the VS Code dev container (`ros_user`) and
`rover-ros2`: use one clone per route.
