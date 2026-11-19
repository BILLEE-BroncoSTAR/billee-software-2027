"""RGB-D input for the arm camera's CV nodes: colour + aligned depth + intrinsics as one frame.

Any CV stage on the RealSense feed (aruco_node now; object detection, grasp-point picking later)
subscribes through RgbdSubscriber and gets an RgbdFrame per colour image:

    self.rgbd = RgbdSubscriber(self, self.on_frame)
    def on_frame(self, frame: RgbdFrame): ...   # frame.color (BGR), frame.depth (m) or None

Topics are relative, so a node started in the `arm_cam` namespace (arm_camera.launch.py) reads
the realsense2_camera node named arm_cam:
    color/image_raw                       sensor_msgs/Image, rgb8
    color/camera_info                     sensor_msgs/CameraInfo
    aligned_depth_to_color/image_raw      sensor_msgs/Image, 16UC1 mm (align_depth.enable)

Pairing: realsense2_camera publishes the colour and the aligned depth image as separate messages.
With enable_sync they carry the same stamp, but either may arrive first. A colour image is handed
on as soon as the depth image stamped within depth_sync_slop of it is here; if that depth hasn't
arrived yet the colour image waits for it, for at most one frame: when the next colour image comes
first, the waiting one goes on without depth (CV stages must cope: aruco_node falls back to
solvePnP). So a frame is never paired with an older depth image, which would put markers in the
wrong place while the arm moves. With no depth for depth_timeout (depth off, or not aligned),
colour images go on straight away. Subscriptions are best effort with a short queue, so a slow
stage drops frames instead of building a backlog.
"""

from __future__ import annotations

import collections
import time
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from arm_vision.aruco import Intrinsics

DEPTH_BUFFER = 6  # depth images kept for pairing: 0.2 s at 30 fps


@dataclass
class RgbdFrame:
    header: Header                 # the colour image's: stamp, frame_id = colour optical frame
    color: np.ndarray              # HxWx3 uint8, BGR
    depth: np.ndarray | None       # HxW float32 metres (0 = no data), aligned to color; or None
    intrinsics: Intrinsics | None  # colour camera K, D; None until camera_info arrives


def stamp_ns(header: Header) -> int:
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def depth_to_metres(image: np.ndarray, encoding: str) -> np.ndarray:
    """16UC1 (mm, RealSense's default) or 32FC1 (m) -> float32 metres."""
    if encoding in ("16UC1", "mono16"):
        return image.astype(np.float32) * 0.001
    if encoding == "32FC1":
        return image.astype(np.float32, copy=False)
    raise ValueError(f"unsupported depth encoding {encoding!r} (expected 16UC1 or 32FC1)")


class RgbdSubscriber:

    def __init__(
        self,
        node: Node,
        callback: Callable[[RgbdFrame], None],
        color_topic: str = "color/image_raw",
        depth_topic: str = "aligned_depth_to_color/image_raw",
        camera_info_topic: str = "color/camera_info",
        use_depth: bool = True,
        depth_sync_slop: float = 0.02,
        depth_timeout: float = 1.0,
    ):
        self._node = node
        self._callback = callback
        self._bridge = CvBridge()
        self._slop_ns = int(depth_sync_slop * 1e9)
        self._depth_timeout = depth_timeout
        self._use_depth = use_depth
        self._depth = collections.deque(maxlen=DEPTH_BUFFER)
        self._last_depth = -float("inf")  # time.monotonic() of the last depth image
        self._pending: Image | None = None  # colour image waiting for its depth image
        self.intrinsics: Intrinsics | None = None
        self._warned = set()
        self.frames = 0
        self.frames_with_depth = 0

        node.create_subscription(Image, color_topic, self._on_color, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, camera_info_topic, self._on_info,
                                 qos_profile_sensor_data)
        if use_depth:
            node.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.topics = {
            "color": node.resolve_topic_name(color_topic),
            "camera_info": node.resolve_topic_name(camera_info_topic),
            "depth": node.resolve_topic_name(depth_topic) if use_depth else None,
        }

    def _on_info(self, msg: CameraInfo):
        intrinsics = Intrinsics.from_camera_info(msg.k, msg.d)
        if intrinsics.valid:
            self.intrinsics = intrinsics
        else:
            self._warn_once("info", f"{self.topics['camera_info']}: K is all zeros (camera not "
                            "calibrated?); no 3D positions until a valid camera_info arrives")

    def _on_color(self, msg: Image):
        if not self._use_depth or time.monotonic() - self._last_depth > self._depth_timeout:
            if self._pending is not None:  # depth stopped while it waited
                pending, self._pending = self._pending, None
                self._emit(pending, None)
            self._emit(msg, None)
            return
        depth = self._find_depth(msg)
        if depth is not None:
            self._emit(msg, depth)
            return
        if self._pending is not None:  # its depth never came: go on without it
            self._emit(self._pending, None)
        self._pending = msg

    def _on_depth(self, msg: Image):
        self._last_depth = time.monotonic()
        self._depth.append(msg)
        if self._pending is not None and self._matches(self._pending, msg):
            pending, self._pending = self._pending, None
            self._emit(pending, msg)

    def _matches(self, color: Image, depth: Image) -> bool:
        return abs(stamp_ns(color.header) - stamp_ns(depth.header)) <= self._slop_ns

    def _find_depth(self, color: Image) -> Image | None:
        for depth in reversed(self._depth):
            if self._matches(color, depth):
                return depth
        return None

    def _emit(self, color_msg: Image, depth_msg: Image | None):
        try:
            color = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self._warn_once("color", f"{self.topics['color']}: can't convert "
                            f"{color_msg.encoding}: {e}")
            return
        depth = (self._convert_depth(depth_msg, color.shape[:2])
                 if depth_msg is not None else None)
        self.frames += 1
        self.frames_with_depth += depth is not None
        self._callback(RgbdFrame(header=color_msg.header, color=color, depth=depth,
                                 intrinsics=self.intrinsics))

    def _convert_depth(self, msg: Image, shape) -> np.ndarray | None:
        if (msg.height, msg.width) != tuple(shape):
            self._warn_once("shape", f"{self.topics['depth']} is {msg.width}x{msg.height} but "
                            f"colour is {shape[1]}x{shape[0]}: is align_depth.enable on? "
                            "Ignoring depth.")
            return None
        try:
            raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            return depth_to_metres(raw, msg.encoding)
        except (CvBridgeError, ValueError) as e:
            self._warn_once("depth", f"{self.topics['depth']}: {e}")
            return None

    def _warn_once(self, key: str, text: str):
        if key not in self._warned:
            self._warned.add(key)
            self._node.get_logger().warning(text)
