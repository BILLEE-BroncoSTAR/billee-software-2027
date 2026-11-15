#!/bin/sh
# Install the ZED udev rule on the HOST, from the one the ZED SDK installer put in this image.
# Without it the ZED's sensor (HID) device is root-only, and the camera fails to open as ros_user
# ("[ZED][MCU] Permissions denied", "NOT VALID SERIAL NUMBER FOR SENSORS MODULE MCU").
#
# Run as root at container start (the l4t devcontainer's postStartCommand). Needs the host's
# rules directory and udev control socket mounted (devcontainer runArgs):
#   /etc/udev/rules.d -> /host/udev/rules.d      /run/udev -> /run/udev
# Does nothing if the host already has an identical rule.
set -eu

src=/etc/udev/rules.d/99-slabs.rules
dst=/host/udev/rules.d/99-slabs.rules

if [ ! -d /host/udev/rules.d ]; then
  echo "zed_udev.sh: host /etc/udev/rules.d is not mounted at /host/udev/rules.d; skipped" >&2
  exit 0
fi
if cmp -s "$src" "$dst"; then
  exit 0
fi
install -m 644 "$src" "$dst"
# udevadm sees the container as a chroot and ignores requests without this. The requests go to the
# host's udevd through /run/udev, so the trigger also applies the rule to an already-plugged ZED.
export SYSTEMD_IGNORE_CHROOT=1
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --subsystem-match=hidraw
echo "zed_udev.sh: installed the ZED udev rule on the host ($dst)"
