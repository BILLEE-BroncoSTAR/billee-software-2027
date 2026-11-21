"""The light patterns, without ROS or a blink(1)."""
from itertools import pairwise

import pytest
from status_light.patterns import BLUE, GREEN, PATTERNS, RED, YELLOW, Player, resolve


def test_modes_map_to_the_agreed_colours():
    assert resolve('teleop-drive') == resolve('teleop-arm') == resolve('teleop-science') == 'teleop'
    assert resolve('astronaut') == resolve('autonomous') == 'autonomous'
    assert PATTERNS['teleop'][0][0] == BLUE
    assert PATTERNS['autonomous'][0][0] == RED
    assert {s[0] for s in PATTERNS['arrived']} == {GREEN, (0, 0, 0)}
    assert PATTERNS['idle'][0][0] == YELLOW


def test_unknown_pattern():
    with pytest.raises(ValueError):
        resolve('disco')


def test_idle_pulses_slowly():
    p = Player('idle')
    t, sent = 0.0, []
    while t < 12.0:
        out = p.step(t)
        if out:
            sent.append((t, out))
        t += 0.05
    # one fade per ~2.8 s: up, down, up, down...
    assert [rgb for _, (rgb, _) in sent][:4] == [YELLOW, (25, 15, 0), YELLOW, (25, 15, 0)]
    gaps = [b[0] - a[0] for a, b in pairwise(sent)]
    assert all(2.7 < g < 2.95 for g in gaps)


def test_arrived_flashes_twice_a_second():
    p = Player('arrived')
    t, sent = 0.0, []
    while t < 2.0:
        out = p.step(t)
        if out:
            sent.append(out)
        t += 0.05
    assert len(sent) == 8 and all(fade == 0 for _, fade in sent)


def test_switching_sends_at_once():
    p = Player('idle')
    p.step(0.0)
    assert p.step(0.1) is None
    p.set('teleop')
    assert p.step(0.15) == (BLUE, 300)
