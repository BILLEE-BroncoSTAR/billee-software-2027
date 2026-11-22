#!/bin/sh
# Install the RoboClaw udev rule (docker/51-roboclaw.rules) on the HOST, so the dev container's
# ros_user can open the USB RoboClaws (/dev/ttyACMn, otherwise root:dialout 0660) for the arm.
#
# Run as root from the repo root at container start (the l4t devcontainer's postStartCommand),
# like blink1_udev.sh. Needs the host's rules directory and udev socket mounted (devcontainer
# runArgs): /etc/udev/rules.d -> /host/udev/rules.d, /run/udev -> /run/udev.
# Does nothing if the host already has an identical rule.
set -eu

src="$(dirname "$0")/51-roboclaw.rules"
dst=/host/udev/rules.d/51-roboclaw.rules

if [ ! -d /host/udev/rules.d ]; then
  echo "roboclaw_udev.sh: host /etc/udev/rules.d is not mounted at /host/udev/rules.d; skipped" >&2
  exit 0
fi
if cmp -s "$src" "$dst"; then
  exit 0
fi
install -m 644 "$src" "$dst"
# See zed_udev.sh: udevadm talks to the host's udevd through /run/udev. The trigger applies the
# rule to RoboClaws already plugged in (permissions are only applied on "add", not "change").
export SYSTEMD_IGNORE_CHROOT=1
udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=tty --sysname-match='ttyACM*'
echo "roboclaw_udev.sh: installed the RoboClaw udev rule on the host ($dst)"
