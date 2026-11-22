#!/bin/sh
# Install the USB-CAN pinning (tooling/80-billee-can.rules + tooling/can-pin) on the HOST, so the
# drivetrain dongle is always can2 and the arm dongle always can3, whatever order they enumerate
# in. Without it they swap between can2 and can3 across replugs and reboots, and the arm or the
# drivetrain comes up on the wrong bus.
#
# Run as root from the repo root at container start (the l4t devcontainer's postStartCommand),
# like blink1_udev.sh. Needs the host's rules directory and udev socket mounted (devcontainer
# runArgs): /etc/udev/rules.d -> /host/udev/rules.d, /run/udev -> /run/udev. The helper goes in
# rules.d next to the rule (udev reads only *.rules there): it is the one host directory mounted.
# Does nothing if the host already has identical files.
set -eu

here="$(cd "$(dirname "$0")/.." && pwd)"
rules=/host/udev/rules.d
rule_src="$here/tooling/80-billee-can.rules"
pin_src="$here/tooling/can-pin"

if [ ! -d "$rules" ]; then
  echo "can_udev.sh: host /etc/udev/rules.d is not mounted at $rules; skipped" >&2
  exit 0
fi
if cmp -s "$rule_src" "$rules/80-billee-can.rules" && cmp -s "$pin_src" "$rules/billee-can-pin"; then
  exit 0
fi
install -m 755 "$pin_src" "$rules/billee-can-pin"
install -m 644 "$rule_src" "$rules/80-billee-can.rules"
# udevadm sees the container as a chroot and ignores requests without this; they go to the
# host's udevd through /run/udev. Re-announcing the CAN interfaces makes the host run the pin
# helper now, so dongles already plugged in are renamed without a replug or reboot.
export SYSTEMD_IGNORE_CHROOT=1
udevadm control --reload-rules
udevadm trigger --action=add --subsystem-match=net --sysname-match='can*'
echo "can_udev.sh: installed the USB-CAN pinning on the host ($rules/80-billee-can.rules)"
