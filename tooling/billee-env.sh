# billee-env.sh — resolve the workspace and Pixi environment for this machine.
#
# Source it (don't execute it). Sets:
#   BILLEE_WS         absolute path to ros2_ws
#   BILLEE_PLATFORM   x86 | wsl | mac | l4t
#   BILLEE_PIXI_ENV   default (x86, wsl) | mac-cpu | l4t
#   BILLEE_ROLE       rover (l4t) | ground (everything else)
#
# BILLEE_ROLE is the deployment split: the Jetson runs the rover stack (real
# drivetrain over CAN, no viewer), every other platform is a ground station
# (viewers, teleop, and the Gazebo sim). tooling/rover-up, ground-up and sim-up
# each refuse the wrong role; set BILLEE_ROLE explicitly to override.
#
# The platform comes from, first match wins:
#   1. $BILLEE_PLATFORM, if already set (per-command override)
#   2. $BILLEE_WS/.billee-platform, written by `make setup <platform>`
#   3. detection: x86_64 -> wsl under WSL2, else x86; aarch64 -> l4t on a Jetson,
#      else mac (the only other ARM64 platform is the Mac's Linux container)
#
# Used by docker/terminal_config.sh (copied into every image), tooling/sim-up,
# tooling/container-init and tooling/desktop-ros2. Safe to source from an interactive shell: no `set -e`/`exit`.

_billee_find_ws() {
  local here dir
  # Beside this file when it lives in the repo's tooling/ dir.
  here="$(cd "$(dirname "${BASH_SOURCE[0]}")" 2>/dev/null && pwd -P)"
  if [[ -f "$here/../ros2_ws/pixi.toml" ]]; then
    (cd "$here/../ros2_ws" && pwd -P)
    return
  fi
  # Walk up from the current directory (repo root, ros2_ws, or anything below).
  dir="$PWD"
  while [[ "$dir" != / ]]; do
    if [[ -f "$dir/pixi.toml" && -d "$dir/src" ]]; then echo "$dir"; return; fi
    if [[ -f "$dir/ros2_ws/pixi.toml" ]]; then echo "$dir/ros2_ws"; return; fi
    dir="$(dirname "$dir")"
  done
  # The containers' mount point (docker/compose.yaml).
  for dir in /workspaces/*/ros2_ws; do
    if [[ -f "$dir/pixi.toml" ]]; then echo "$dir"; return; fi
  done
}

_billee_detect_platform() {
  case "$(uname -m)" in
    x86_64)
      if [[ "$(uname -r)" == *[Mm]icrosoft* ]]; then echo wsl; else echo x86; fi ;;
    aarch64 | arm64)
      if [[ -f /etc/nv_tegra_release ]]; then echo l4t; else echo mac; fi ;;
    *) echo unknown ;;
  esac
}

BILLEE_WS="${BILLEE_WS:-$(_billee_find_ws)}"

if [[ -z "${BILLEE_PLATFORM:-}" && -n "$BILLEE_WS" && -f "$BILLEE_WS/.billee-platform" ]]; then
  read -r BILLEE_PLATFORM < "$BILLEE_WS/.billee-platform"
fi
BILLEE_PLATFORM="${BILLEE_PLATFORM:-$(_billee_detect_platform)}"

case "$BILLEE_PLATFORM" in
  mac) BILLEE_PIXI_ENV=mac-cpu ;;
  l4t) BILLEE_PIXI_ENV=l4t ;;
  *) BILLEE_PIXI_ENV=default ;;
esac

# Deployment role. Only the Jetson is the rover; everything else is a ground station.
if [[ -z "${BILLEE_ROLE:-}" ]]; then
  if [[ "$BILLEE_PLATFORM" == l4t ]]; then BILLEE_ROLE=rover; else BILLEE_ROLE=ground; fi
fi

export BILLEE_WS BILLEE_PLATFORM BILLEE_PIXI_ENV BILLEE_ROLE
unset -f _billee_find_ws _billee_detect_platform
