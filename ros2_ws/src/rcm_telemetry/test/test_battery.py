"""rcm_telemetry.battery: charge from the RCM's INA780B voltages, against a fake GDS."""

import math
import os
import sys

import pytest
from rcm_telemetry.battery import (
    GdsClient,
    apply_update,
    charge_fraction,
    default_packs,
)

sys.path.insert(0, os.path.dirname(__file__))
from fake_gds import reading, serve_in_thread


def test_charge_fraction_between_empty_and_full():
    assert charge_fraction(13.6, 13.6, 16.8) == 0.0
    assert charge_fraction(16.8, 13.6, 16.8) == 1.0
    assert charge_fraction(15.2, 13.6, 16.8) == pytest.approx(0.5)
    assert charge_fraction(12.0, 13.6, 16.8) == 0.0   # below the undervoltage bound
    assert charge_fraction(17.0, 13.6, 16.8) == 1.0   # still settling after a charge
    assert math.isnan(charge_fraction(math.nan, 13.6, 16.8))


def test_pack_full_voltages_from_cell_counts():
    packs = default_packs()
    assert packs["drive"].full_voltage(4.2) == pytest.approx(16.8)
    for name in ("logic", "science", "arm"):
        assert packs[name].full_voltage(4.2) == pytest.approx(25.2)
        assert packs[name].empty_voltage == 20.4      # fprime-billee lucadev's default
    assert packs["drive"].empty_voltage == 13.6


def test_drive_pack_uses_the_highest_valid_channel():
    packs = default_packs()
    apply_update(packs, "rp2350Deployment.inaManager.DRIVE1_POWER", reading(0.0, 0.0), 0.0)
    apply_update(packs, "rp2350Deployment.inaManager.DRIVE2_POWER", reading(15.2, 2.0), 0.0)
    apply_update(packs, "rp2350Deployment.inaManager.DRIVE3_POWER", reading(16.5, 1.0, False), 0.0)
    drive = packs["drive"]
    assert drive.voltage == pytest.approx(15.2)        # the invalid 16.5 V is ignored
    assert drive.current == pytest.approx(2.0)
    assert drive.percentage(4.2) == pytest.approx(0.5)


def test_invalid_only_reading_leaves_the_pack_unknown():
    packs = default_packs()
    apply_update(packs, "rp2350Deployment.inaManager.SCIENCE_POWER", reading(0.0, 0.0, False), 0.0)
    assert math.isnan(packs["science"].percentage(4.2))


def test_live_bounds_move_the_empty_voltage():
    packs = default_packs()
    apply_update(packs, "rp2350Deployment.fpManager.ARM_POWER_BOUNDS",
                 {"vbusFaultLow": 21.0, "vbusFaultHigh": 35.0, "currentFaultHigh": 100.0}, 0.0)
    assert packs["arm"].empty_voltage == 21.0
    assert packs["logic"].empty_voltage == 20.4


def test_client_against_a_fake_gds():
    server = serve_in_thread()
    try:
        client = GdsClient(f"http://127.0.0.1:{server.server_address[1]}")
        packs = default_packs()
        for name, val in client.poll():
            apply_update(packs, name, val, 0.0)
        assert client.session == "fake-session"
        assert packs["drive"].percentage(4.2) == pytest.approx(0.5)
        assert packs["logic"].percentage(4.2) == pytest.approx(0.5)
        assert packs["arm"].percentage(4.2) == pytest.approx(1.0)
        assert math.isnan(packs["science"].percentage(4.2))  # its sensor isn't answering
    finally:
        server.shutdown()


def test_client_unreachable_raises_oserror():
    with pytest.raises(OSError):
        GdsClient("http://127.0.0.1:9", timeout=0.5).poll()
