"""The mode light's patterns: what the blink(1) shows in each state of the rover state machine.

    idle        yellow, slowly pulsing       dashboard up, nothing running
    teleop      solid blue                   Teleop · Drive, Arm, Science
    autonomous  solid red                    Autonomous, Astronaut
    arrived     flashing green               the autonomous mode reached its goal (publish it on
                                             /status_light/pattern when that exists)
    off         dark

A pattern is a cycle of steps (rgb, fade_ms, hold_s): fade to rgb over fade_ms, then wait hold_s
before the next step. Solid colours are one step, re-sent every hold_s so a light that was
unplugged and plugged back in picks the colour up again. No ROS or USB here, so it is testable
anywhere.
"""

YELLOW = (255, 150, 0)
YELLOW_DIM = (25, 15, 0)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLACK = (0, 0, 0)

PATTERNS = {
    # ~3 s per breath: fade up 1.3 s, hold, fade down 1.3 s, hold.
    'idle': [(YELLOW, 1300, 1.5), (YELLOW_DIM, 1300, 1.5)],
    'teleop': [(BLUE, 300, 2.0)],
    'autonomous': [(RED, 300, 2.0)],
    # 2 flashes a second.
    'arrived': [(GREEN, 0, 0.25), (BLACK, 0, 0.25)],
    'off': [(BLACK, 300, 2.0)],
}

# Names a caller may also use: the state machine's modes map onto the patterns above.
ALIASES = {
    'teleop-drive': 'teleop', 'teleop-arm': 'teleop', 'teleop-science': 'teleop',
    'drive': 'teleop', 'arm': 'teleop', 'science': 'teleop',
    'astronaut': 'autonomous', 'auto': 'autonomous',
    'success': 'arrived', 'goal': 'arrived',
    'none': 'off',
}


def resolve(name):
    """A pattern name (or alias) -> its canonical name. ValueError if unknown."""
    key = (name or '').strip().lower()
    key = ALIASES.get(key, key)
    if key not in PATTERNS:
        known = ', '.join(sorted(PATTERNS))
        raise ValueError(f"unknown light pattern '{name}' (use {known})")
    return key


class Player:
    """Steps through a pattern against a clock. step(now) returns the (rgb, fade_ms) to send
    now, or None while the current step is still holding."""

    def __init__(self, pattern='idle'):
        self.set(pattern)

    def set(self, pattern):
        self.pattern = resolve(pattern)
        self.index = 0
        self.next_at = 0.0  # send at once

    def step(self, now):
        if now < self.next_at:
            return None
        steps = PATTERNS[self.pattern]
        rgb, fade_ms, hold_s = steps[self.index % len(steps)]
        self.index = (self.index + 1) % len(steps)
        self.next_at = now + fade_ms / 1000.0 + hold_s
        return rgb, fade_ms

    def resend(self):
        """Send the current step again at once (e.g. after the light was reconnected)."""
        steps = PATTERNS[self.pattern]
        self.index = (self.index - 1) % len(steps)
        self.next_at = 0.0
