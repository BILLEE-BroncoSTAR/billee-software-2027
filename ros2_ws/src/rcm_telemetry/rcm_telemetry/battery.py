"""Battery charge of the RCM's four packs from its F´ GDS telemetry (no ROS here: testable alone).

The Rover Control Module (billee-hardware-2027, fprime-billee-rcm) measures each pack with an
INA780B: one per drivetrain wheel channel (DRIVE1..6, all on the DRIVE pack), and one each for
ARM, SCIENCE and LOGIC. Its F´ GDS on the Jetson host serves that telemetry over HTTP
(fprime_gds' Flask API, the web UI on :5000):

    GET /session                         -> {"session": key}
    GET /dictionary/channels             -> {"dictionary": {id: {"full_name": ...}}}
    GET /channels?session=<key>          -> {"history": [{"id", "val", ...}]}, new since the last call

Charge is estimated from voltage, linearly from the pack's undervoltage fault bound (0 %) to a
full charge, cells x full_cell_voltage (100 %). The bounds are fpManager's parameters
(fprime-billee, Components/FPManager/FPManager.fpp: 13.6 V DRIVE 4S, 20.4 V for the 6S packs:
3.4 V/cell), read live from the RCM's *_POWER_BOUNDS telemetry, these values until it arrives.
Not vbusFaultHigh (35 V): that's an overvoltage guard, far above a full pack (16.8 / 25.2 V).
Voltage sags under load, so the estimate dips while a subsystem works and recovers at rest.
"""

from __future__ import annotations

import json
import math
import time
import urllib.parse
import urllib.request
from dataclasses import dataclass, field


@dataclass
class Pack:
    name: str              # drive | logic | science | arm
    cells: int
    empty_voltage: float   # V, 0 %: the undervoltage fault bound, until the RCM reports its own
    channels: list[str]    # INA780B power channels (full names end with these)
    bounds_channel: str    # fpManager.<SUB>_POWER_BOUNDS
    voltage: float = math.nan
    current: float = math.nan
    power: float = math.nan
    updated: float = -math.inf   # time.monotonic() of the last valid reading
    readings: dict[str, dict] = field(default_factory=dict)

    def full_voltage(self, full_cell_voltage: float) -> float:
        return self.cells * full_cell_voltage

    def percentage(self, full_cell_voltage: float) -> float:
        """0..1, or NaN without a valid reading."""
        return charge_fraction(self.voltage, self.empty_voltage,
                               self.full_voltage(full_cell_voltage))


def default_packs() -> dict[str, Pack]:
    return {
        "drive": Pack("drive", 4, 13.6, [f"inaManager.DRIVE{i}_POWER" for i in range(1, 7)],
                      "fpManager.DRIVETRAIN_POWER_BOUNDS"),
        "logic": Pack("logic", 6, 20.4, ["inaManager.LOGIC_POWER"],
                      "fpManager.LOGIC_POWER_BOUNDS"),
        "science": Pack("science", 6, 20.4, ["inaManager.SCIENCE_POWER"],
                        "fpManager.SCIENCE_POWER_BOUNDS"),
        "arm": Pack("arm", 6, 20.4, ["inaManager.ARM_POWER"], "fpManager.ARM_POWER_BOUNDS"),
    }


def charge_fraction(voltage: float, empty: float, full: float) -> float:
    if not math.isfinite(voltage) or full <= empty:
        return math.nan
    return min(1.0, max(0.0, (voltage - empty) / (full - empty)))


def apply_update(packs: dict[str, Pack], name: str, val, now: float) -> None:
    """One telemetry update (channel full name, value) into the packs it belongs to."""
    for pack in packs.values():
        if name.endswith(pack.bounds_channel) and isinstance(val, dict):
            low = val.get("vbusFaultLow")
            if isinstance(low, (int, float)) and math.isfinite(low):
                pack.empty_voltage = float(low)
            return
        for channel in pack.channels:
            if name.endswith(channel) and isinstance(val, dict):
                pack.readings[channel] = {**val, "_at": now}
                _combine(pack, now)
                return


def _combine(pack: Pack, now: float, max_age: float = 5.0) -> None:
    """Pack voltage = the highest valid channel (a switched-off DRIVE channel reads ~0 V, the
    others still see the pack); current and power = their sums."""
    valid = [r for r in pack.readings.values()
             if r.get("valid") and now - r["_at"] <= max_age]
    if not valid:
        return
    pack.voltage = max(float(r.get("voltage", math.nan)) for r in valid)
    pack.current = sum(float(r.get("current", 0.0)) for r in valid)
    pack.power = sum(float(r.get("power", 0.0)) for r in valid)
    pack.updated = now


class GdsClient:
    """The F´ GDS's HTTP API. Raises OSError / ValueError on connection or format problems."""

    def __init__(self, url: str = "http://localhost:5000", timeout: float = 2.0):
        self.url = url.rstrip("/")
        self.timeout = timeout
        self.session: str | None = None
        self.names: dict[int, str] = {}

    def _get(self, path: str, **query):
        q = f"?{urllib.parse.urlencode(query)}" if query else ""
        with urllib.request.urlopen(self.url + path + q, timeout=self.timeout) as r:
            return json.load(r)

    def connect(self) -> None:
        self.names = {int(k): v["full_name"]
                      for k, v in self._get("/dictionary/channels")["dictionary"].items()}
        s = self._get("/session")
        self.session = s.get("session") if isinstance(s, dict) else str(s)

    def poll(self) -> list[tuple[str, object]]:
        """(channel full name, value) for every update since the last poll."""
        if self.session is None:
            self.connect()
        history = self._get("/channels", session=self.session).get("history", [])
        return [(self.names.get(int(u["id"]), str(u["id"])), u.get("val")) for u in history]


def now() -> float:
    return time.monotonic()
