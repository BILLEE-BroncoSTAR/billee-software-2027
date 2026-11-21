# Tooling scripts (`tooling/`)

Scripts run from the repository root. Only the CAN ones touch the rover's hardware.

| Script | Runs on | Does |
|---|---|---|
| [`can-up`](#can-up) | Jetson **host** | brings a CAN interface up or down |
| [`can2.service`](#can2service) | Jetson **host** (systemd) | brings `can2` up at every boot |
| [`argus-watchdog`, `argus-watchdog.service`](#argus-watchdog) | Jetson **host** (systemd) | restarts `nvargus-daemon` when it's down, wedged or leaking camera sessions |
| [`can-clear-faults`](#can-clear-faults) | host or container | clears latched ODESC errors, optionally reboots a board |
| [`sim-up`](#sim-up) | container | one-command Gazebo sim |
| [`desktop-ros2`, `rover-ros2`](#desktop-ros2-rover-ros2) | host | build and run the dev images without VS Code |

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

## argus-watchdog

One flaky CSI camera can take every camera down: its capture times out in hardware and
`nvargus-daemon` (Argus, on the host) crashes, stops accepting clients, or leaves sessions
allocated. systemd restarts a crash by itself; the rest needed `sudo systemctl restart
nvargus-daemon` by hand. This does it automatically. Python standard library, host only.

```bash
sudo install -m 755 tooling/argus-watchdog /usr/local/bin/argus-watchdog
sudo cp tooling/argus-watchdog.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now argus-watchdog.service
journalctl -u argus-watchdog -f             # what it saw and did

tooling/argus-watchdog --status             # Argus state and its accept queue, no root needed
sudo tooling/argus-watchdog --dry-run       # detect and log, never restart
```

After changing `tooling/argus-watchdog`, run the `install` line again and
`sudo systemctl restart argus-watchdog`.

| Restarts Argus when | Detected by |
|---|---|
| it stops accepting clients | connections queued on `/tmp/argus_socket` for 10 s (`--wedge-s`), read from `/proc/net/unix` (it never connects, so it costs Argus nothing) |
| sessions leaked by a client that died mid-close | 3 `AlreadyAllocated … is in use` in its log within 60 s (`--leak-count`) |
| the service is down and systemd gave up on it | not `active` for 15 s (`--inactive-s`) |

- A **camera error** (`Sending critical error event`, `Camera HwEvents wait`) is only logged:
  Argus usually survives it and only that camera's session dies, so a restart would drop the
  healthy feeds too.
- Restarts are at least 30 s apart; every restart within 10 min of the last doubles that, up to
  5 min, so a bad cable can't make all the feeds flap. Ten quiet minutes reset it.
- The camera nodes recover by themselves after any Argus restart (theirs or this one's): a
  pipeline with no frame for 5 s is restarted ([cameras.md](cameras.md)). Feeds are back about
  30 s after a crash.

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
tooling/desktop-ros2 build|run|shell [args...]   # x86: docker/Dockerfile.desktop.humble, --gpus all
tooling/rover-ros2   build|run|shell [args...]   # Jetson: docker/Dockerfile.l4t-humble, --runtime nvidia
```

- `build` passes extra arguments to `docker build` (e.g. `--build-arg OPENPOSE_JOBS=3`).
- `run` starts the image's default command, or the extra arguments instead
  (`tooling/rover-ros2 run pixi run --environment l4t build`).
- `shell` opens Bash, bypassing the entrypoint.

Both mount only `ros2_ws` (at `/home/ros_user/ros2_ws`), not the whole repository, so
`tooling/` and `configs/` aren't inside. Unlike the l4t dev container, `rover-ros2` doesn't run
the `postStartCommand` steps (GPU groups, ZED udev rule and calibration, Argus socket), so the ZED
and the CSI cameras may not work in it.
