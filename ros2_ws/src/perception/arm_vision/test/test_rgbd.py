"""RgbdSubscriber's colour / depth pairing, driven directly (no DDS).

A colour image goes on with the depth image carrying its stamp, whichever arrives first, and
never with an older one.
"""

import numpy as np
import pytest

pytest.importorskip("cv_bridge")
from arm_vision import rgbd
from arm_vision.rgbd import RgbdSubscriber, depth_to_metres
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image
from std_msgs.msg import Header

W, H = 8, 6


class FakeNode:
    """Just what RgbdSubscriber calls on its node."""

    def __init__(self):
        self.callbacks = {}

    def create_subscription(self, _type, topic, callback, _qos):
        self.callbacks[topic] = callback

    def resolve_topic_name(self, topic):
        return "/arm_cam/" + topic

    def get_logger(self):
        class Log:
            def warning(self, _text):
                pass
        return Log()


def header(ms):
    return Header(stamp=Time(sec=ms // 1000, nanosec=(ms % 1000) * 1_000_000), frame_id="c")


def color(ms):
    return Image(header=header(ms), height=H, width=W, encoding="rgb8", step=W * 3,
                 data=bytes([ms % 256]) * (W * H * 3))


def depth(ms, mm=600, width=W):
    data = np.full((H, width), mm, dtype=np.uint16).tobytes()
    return Image(header=header(ms), height=H, width=width, encoding="16UC1", step=width * 2,
                 data=data)


@pytest.fixture
def sub(monkeypatch):
    clock = {"t": 100.0}
    monkeypatch.setattr(rgbd.time, "monotonic", lambda: clock["t"])
    node, frames = FakeNode(), []
    s = RgbdSubscriber(node, frames.append)
    s.on_color = node.callbacks["color/image_raw"]
    s.on_depth = node.callbacks["aligned_depth_to_color/image_raw"]
    s.clock, s.out = clock, frames
    return s


def stamps(frames):
    return [(f.header.stamp.sec * 1000 + f.header.stamp.nanosec // 1_000_000, f.depth is not None)
            for f in frames]


def test_depth_first_then_colour(sub):
    sub.on_depth(depth(0))
    sub.on_color(color(0))
    assert stamps(sub.out) == [(0, True)]
    assert sub.out[0].depth[0, 0] == pytest.approx(0.6)
    assert sub.out[0].color.shape == (H, W, 3)


def test_colour_first_waits_for_its_depth(sub):
    sub.on_depth(depth(0))
    sub.on_color(color(0))
    sub.on_color(color(33))       # its depth isn't here yet: must not take depth(0)
    assert stamps(sub.out) == [(0, True)]
    sub.on_depth(depth(33))
    assert stamps(sub.out) == [(0, True), (33, True)]


def test_missing_depth_goes_on_without_it_next_frame(sub):
    sub.on_depth(depth(0))
    sub.on_color(color(0))
    sub.on_color(color(33))       # depth(33) lost
    sub.on_color(color(66))
    assert stamps(sub.out) == [(0, True), (33, False)]
    sub.on_depth(depth(66))
    assert stamps(sub.out) == [(0, True), (33, False), (66, True)]


def test_no_depth_stream_means_no_waiting(sub):
    sub.on_color(color(0))        # depth never seen
    assert stamps(sub.out) == [(0, False)]
    sub.on_depth(depth(33))
    sub.clock["t"] += 5.0          # depth stopped
    sub.on_color(color(5000))
    assert stamps(sub.out) == [(0, False), (5000, False)]


def test_waiting_frame_goes_on_when_depth_stops(sub):
    sub.on_depth(depth(0))
    sub.on_color(color(0))
    sub.on_color(color(33))       # waits for depth(33)
    sub.clock["t"] += 5.0          # ...which never comes: depth stopped
    sub.on_color(color(66))
    assert stamps(sub.out) == [(0, True), (33, False), (66, False)]


def test_depth_of_wrong_size_is_ignored(sub):
    sub.on_depth(depth(0, width=W + 2))
    sub.on_color(color(0))
    assert stamps(sub.out) == [(0, False)]


def test_depth_to_metres():
    assert depth_to_metres(np.array([[1500]], np.uint16), "16UC1")[0, 0] == pytest.approx(1.5)
    assert depth_to_metres(np.array([[1.25]], np.float32), "32FC1")[0, 0] == pytest.approx(1.25)
    with pytest.raises(ValueError):
        depth_to_metres(np.zeros((1, 1), np.uint8), "mono8")
