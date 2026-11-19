#!/bin/sh
# Install Intel's RealSense udev rule on the HOST, from the copy in this image
# (docker/99-realsense-libusb.rules). librealsense here talks to the camera through libusb (RSUSB
# backend, how conda-forge builds it), which needs read/write on the camera's /dev/bus/usb node;
# without the rule that node is root-only and realsense2_camera can't open the D435i as ros_user
# ("failed to open usb interface", "No RealSense devices were found").
#
# Run as root at container start (the l4t devcontainer's postStartCommand), like zed_udev.sh, with
# the host's rules directory and udev control socket mounted (devcontainer runArgs):
#   /etc/udev/rules.d -> /host/udev/rules.d      /run/udev -> /run/udev
# Does nothing if the host already has an identical rule.
set -eu

src=/etc/udev/rules.d/99-realsense-libusb.rules
dst=/host/udev/rules.d/99-realsense-libusb.rules

if [ ! -d /host/udev/rules.d ]; then
  echo "realsense_udev.sh: host /etc/udev/rules.d is not mounted at /host/udev/rules.d; skipped" >&2
  exit 0
fi
if cmp -s "$src" "$dst"; then
  exit 0
fi
install -m 644 "$src" "$dst"
# udevadm sees the container as a chroot and ignores requests without this. The requests go to the
# host's udevd through /run/udev, so the trigger also applies the rule to an already-plugged camera
# (iio / hidraw: the IMU, when the kernel's HID sensor driver has it instead of libusb).
export SYSTEMD_IGNORE_CHROOT=1
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --subsystem-match=hidraw --subsystem-match=iio
echo "realsense_udev.sh: installed the RealSense udev rule on the host ($dst)"
