"""Which Intel RealSense cameras are plugged in, from sysfs (no librealsense, no device access).

Used at launch time (arm_camera.launch.py, rover.launch.py arm_camera:=auto) to decide whether to
start the camera and to warn about a USB 2 link, before realsense2_camera opens anything.
"""

from __future__ import annotations

import glob
import os
from dataclasses import dataclass

INTEL = "8086"
# librealsense v2.57.7 src/ds/d400/d400-private.h. Recovery = firmware update (DFU) mode: the
# camera shows up like this after a failed rs-fw-update and streams nothing.
PRODUCTS = {
    "0ad3": "D415",
    "0b07": "D435",
    "0b3a": "D435i",
    "0b4b": "D430i",
    "0b5b": "D405",
    "0b5c": "D455",
    "1156": "D436",
    "0adb": "D400 (recovery mode)",
    "0adc": "D400 USB2 (recovery mode)",
}


@dataclass
class UsbCamera:
    model: str
    product_id: str
    serial: str
    port: str         # sysfs device name, e.g. "2-1.3" (bus-port.port)
    speed_mbps: int   # 5000 / 10000 = USB 3, 480 = USB 2

    @property
    def usb3(self) -> bool:
        return self.speed_mbps >= 5000

    @property
    def recovery(self) -> bool:
        return self.product_id in ("0adb", "0adc")

    def describe(self) -> str:
        link = f"USB {'3' if self.usb3 else '2'} ({self.speed_mbps} Mb/s)"
        return f"RealSense {self.model} serial {self.serial or '?'} on port {self.port}, {link}"


def _read(path: str) -> str:
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ""


def realsense_cameras(sysfs: str = "/sys/bus/usb/devices") -> list[UsbCamera]:
    cameras = []
    for dev in sorted(glob.glob(os.path.join(sysfs, "*"))):
        if _read(os.path.join(dev, "idVendor")) != INTEL:
            continue
        pid = _read(os.path.join(dev, "idProduct")).lower()
        if pid not in PRODUCTS:
            continue
        try:
            speed = int(float(_read(os.path.join(dev, "speed")) or 0))
        except ValueError:
            speed = 0
        cameras.append(UsbCamera(model=PRODUCTS[pid], product_id=pid,
                                 serial=_read(os.path.join(dev, "serial")),
                                 port=os.path.basename(dev), speed_mbps=speed))
    return cameras


def camera_warnings(cameras: list[UsbCamera]) -> list[str]:
    warnings = []
    for c in cameras:
        if c.recovery:
            warnings.append(f"{c.describe()}: in recovery (firmware update) mode, it won't "
                            "stream. Flash firmware with rs-fw-update (docs/arm-camera.md).")
        elif not c.usb3:
            warnings.append(f"{c.describe()}: USB 2 link. Colour + depth at 30 fps needs USB 3: "
                            "use a USB 3 port and cable (blue / SS), plug the connector in fully.")
    return warnings
