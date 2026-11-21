#!/bin/sh
# Install the blink(1) udev rule (docker/51-blink1.rules) on the HOST, so the mode light
# (ros2_ws/src/status_light) can be opened as ros_user. Without it the light's hidraw device is
# root-only and the node logs "no blink(1)".
#
# Run as root from the repo root at container start (the l4t devcontainer's postStartCommand),
# like zed_udev.sh. Needs the host's rules directory and udev socket mounted (devcontainer
# runArgs): /etc/udev/rules.d -> /host/udev/rules.d, /run/udev -> /run/udev.
# Does nothing if the host already has an identical rule.
set -eu

src="$(dirname "$0")/51-blink1.rules"
dst=/host/udev/rules.d/51-blink1.rules

if [ ! -d /host/udev/rules.d ]; then
  echo "blink1_udev.sh: host /etc/udev/rules.d is not mounted at /host/udev/rules.d; skipped" >&2
  exit 0
fi
if cmp -s "$src" "$dst"; then
  exit 0
fi
install -m 644 "$src" "$dst"
# See zed_udev.sh: udevadm talks to the host's udevd through /run/udev.
export SYSTEMD_IGNORE_CHROOT=1
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --subsystem-match=hidraw
echo "blink1_udev.sh: installed the blink(1) udev rule on the host ($dst)"
