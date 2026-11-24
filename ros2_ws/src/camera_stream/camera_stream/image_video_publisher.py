"""A ROS image topic -> H.264 -> foxglove_msgs/CompressedVideo: a smooth, low-bandwidth feed.

    ros2 run camera_stream image_video_publisher --ros-args -r __ns:=/arm_cam \
        -p image_topic:=color/image_raw -p video_topic:=color/video

For cameras whose frames arrive as ROS images (the arm's RealSense), where the CSI cameras get
H.264 straight from their GStreamer pipeline (csi_compressed_video_publisher.py). Every camera
frame is encoded, independently of any CV node, so the video stays at the camera's frame rate
however long a detection takes; overlays (ArUco markers, the keyboard) are drawn on top by
Foxglove from their own topics.

Encoder (`encoder`):
  auto   the Jetson's hardware encoder (nvv4l2h264enc, system GStreamer) when it has one (Orin
         AGX / NX: /dev/v4l2-nvenc), else x264 in software (pixi's ffmpeg). The Orin Nano has no
         hardware encoder; x264 ultrafast takes ~14 % of one of its cores for 640x480 @ 30.
  nvenc  hardware only.          x264  software only.
Both: no B-frames, an access unit delimiter and SPS/PPS before every keyframe (every `gop`
frames), so a viewer can join at any keyframe and the byte stream splits into frames at the
delimiters. Bitrate-capped (`bitrate`, bits/s): 1 Mbit/s by default, for the radio link.

A frame that arrives while the encoder is still busy replaces the waiting one (latest wins), so
a slow moment drops a frame rather than building a backlog. The encoder is restarted if it
exits. Each output frame carries its input image's stamp, or with `restamp` (mode:=sim, see
aruco_node) this node's clock when the frame went in.
"""

from __future__ import annotations

import collections
import os
import shutil
import subprocess
import threading
import time

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from camera_stream.csi_compressed_video_publisher import AUD, split_access_units

try:
    from foxglove_msgs.msg import CompressedVideo
except ImportError:  # reported in __init__
    CompressedVideo = None

NVENC_DEVICE = "/dev/v4l2-nvenc"
GST_LAUNCH = "/usr/bin/gst-launch-1.0"
GST_PLUGIN_DIR = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
# Raw formats this node feeds the encoders: ROS encoding -> (ffmpeg pix_fmt, GStreamer format,
# bytes per pixel).
FORMATS = {"rgb8": ("rgb24", "RGB", 3), "bgr8": ("bgr24", "BGR", 3),
           "mono8": ("gray", "GRAY8", 1)}


def hardware_encoder_available() -> bool:
    """nvv4l2h264enc can run: the device node exists and the plugin is installed."""
    return (os.path.exists(NVENC_DEVICE) and os.path.exists(GST_LAUNCH)
            and os.path.exists(os.path.join(GST_PLUGIN_DIR, "libgstnvvideo4linux2.so")))


def find_ffmpeg() -> str | None:
    prefix = os.environ.get("CONDA_PREFIX", "")
    pixi = os.path.join(prefix, "bin", "ffmpeg") if prefix else ""
    return pixi if pixi and os.access(pixi, os.X_OK) else shutil.which("ffmpeg")


def encoder_command(encoder: str, width: int, height: int, ros_encoding: str, fps: int,
                    bitrate: int, gop: int, ffmpeg: str | None = None) -> list[str]:
    """argv of an encoder reading raw frames on stdin and writing H.264 (Annex B) to stdout."""
    pix_fmt, gst_format, _ = FORMATS[ros_encoding]
    if encoder == "nvenc":
        return [GST_LAUNCH, "-q", "fdsrc", "fd=0", "!",
                "rawvideoparse", f"width={width}", f"height={height}", f"format={gst_format.lower()}",
                f"framerate={fps}/1", "!", "videoconvert", "!", "video/x-raw,format=I420", "!",
                "nvvidconv", "!", "video/x-raw(memory:NVMM),format=NV12", "!",
                "nvv4l2h264enc", "profile=4", f"bitrate={bitrate}", f"peak-bitrate={bitrate}",
                "control-rate=0", "preset-level=1", f"iframeinterval={gop}", f"idrinterval={gop}",
                "insert-sps-pps=true", "insert-aud=true", "num-B-Frames=0", "maxperf-enable=true",
                "!", "video/x-h264,stream-format=byte-stream", "!", "fdsink", "fd=1", "sync=false"]
    if encoder == "x264":
        return [ffmpeg or "ffmpeg", "-hide_banner", "-loglevel", "error", "-nostdin",
                "-f", "rawvideo", "-pix_fmt", pix_fmt, "-s", f"{width}x{height}", "-r", str(fps),
                "-i", "pipe:0", "-fps_mode", "passthrough",
                "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
                "-x264-params", (f"aud=1:repeat-headers=1:keyint={gop}:min-keyint={gop}"
                                 ":scenecut=0:bframes=0"),
                "-b:v", str(bitrate), "-maxrate", str(bitrate), "-bufsize", str(bitrate // 2),
                "-pix_fmt", "yuv420p", "-threads", "1", "-f", "h264", "pipe:1"]
    raise ValueError(f"encoder must be nvenc or x264, not {encoder!r}")


class ImageVideoPublisher(Node):

    def __init__(self) -> None:
        super().__init__("image_video_publisher")
        p = self.declare_parameter
        self.image_topic = str(p("image_topic", "color/image_raw").value)
        self.video_topic = str(p("video_topic", "color/video").value)
        self.encoder_choice = str(p("encoder", "auto").value)
        self.fps = int(p("fps", 30).value)  # rate control only; frames go out as they come
        self.bitrate = int(p("bitrate", 1_000_000).value)
        self.gop = int(p("gop", 30).value)
        self.restamp = bool(p("restamp", False).value)
        self.frame_id = str(p("frame_id", "").value)  # "" = the image's

        if CompressedVideo is None:
            raise RuntimeError("foxglove_msgs isn't built: pixi run build --packages-select "
                               "foxglove_msgs (in third_party)")
        if self.encoder_choice not in ("auto", "nvenc", "x264"):
            raise ValueError(f"encoder must be auto, nvenc or x264, not {self.encoder_choice!r}")
        self.ffmpeg = find_ffmpeg()
        self.encoder = self._pick_encoder()

        self.pub = self.create_publisher(CompressedVideo, self.video_topic, 10)
        self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)

        self._lock = threading.Condition()
        self._pending: tuple | None = None  # (raw bytes, stamp, frame_id): the next frame in
        self._stamps: collections.deque = collections.deque(maxlen=120)  # frames in the encoder
        self._proc: subprocess.Popen | None = None
        self._shape: tuple | None = None  # (width, height, encoding) the encoder runs at
        self._next_start = 0.0
        self._stop = False
        self._stats = {"in": 0, "out": 0, "dropped": 0, "bytes": 0, "since": time.monotonic()}
        threading.Thread(target=self._writer, daemon=True).start()
        self.create_timer(10.0, self._log_stats)
        self.get_logger().info(
            f"{self.resolve_topic_name(self.image_topic)} -> "
            f"{self.resolve_topic_name(self.video_topic)}: H.264 via {self.encoder}, "
            f"{self.bitrate / 1e6:g} Mbit/s cap, keyframe every {self.gop} frames")

    def _pick_encoder(self) -> str:
        if self.encoder_choice == "nvenc" or (self.encoder_choice == "auto"
                                              and hardware_encoder_available()):
            return "nvenc"
        if not self.ffmpeg:
            raise RuntimeError("no H.264 encoder: no hardware encoder and no ffmpeg")
        return "x264"

    # ---- input --------------------------------------------------------------------------------

    def on_image(self, msg: Image) -> None:
        if msg.encoding not in FORMATS:
            self.get_logger().error(f"unsupported image encoding {msg.encoding!r} "
                                    f"(expected one of {', '.join(FORMATS)})", once=True)
            return
        bpp = FORMATS[msg.encoding][2]
        row = msg.width * bpp
        data = bytes(msg.data)
        if msg.step != row:  # padded rows: pack them
            data = b"".join(data[r * msg.step: r * msg.step + row] for r in range(msg.height))
        if self.restamp:
            now = self.get_clock().now()
            if now.nanoseconds == 0:  # use_sim_time, no /clock yet
                return
            stamp = now.to_msg()
        else:
            stamp = msg.header.stamp
        frame = (data, (msg.width, msg.height, msg.encoding), stamp,
                 self.frame_id or msg.header.frame_id)
        with self._lock:
            self._stats["in"] += 1
            if self._pending is not None:
                self._stats["dropped"] += 1  # the encoder fell behind: latest frame wins
            self._pending = frame
            self._lock.notify()

    # ---- encoder ------------------------------------------------------------------------------

    def _start(self, shape) -> bool:
        width, height, encoding = shape
        argv = encoder_command(self.encoder, width, height, encoding, self.fps, self.bitrate,
                               self.gop, self.ffmpeg)
        try:
            self._proc = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                          stderr=subprocess.PIPE, bufsize=0)
        except OSError as e:
            self.get_logger().error(f"can't start the {self.encoder} encoder: {e}")
            return False
        self._shape = shape
        self._stamps.clear()
        threading.Thread(target=self._reader, args=(self._proc,), daemon=True).start()
        threading.Thread(target=self._stderr, args=(self._proc,), daemon=True).start()
        self.get_logger().info(f"encoding {width}x{height} {encoding} with {self.encoder}")
        return True

    def _stop_encoder(self) -> None:
        proc, self._proc = self._proc, None
        if proc is None:
            return
        try:
            proc.stdin.close()
            proc.wait(timeout=2)
        except (OSError, subprocess.TimeoutExpired):
            proc.kill()

    def _writer(self) -> None:
        while True:
            with self._lock:
                while self._pending is None and not self._stop:
                    self._lock.wait()
                if self._stop:
                    return
                data, shape, stamp, frame_id = self._pending
                self._pending = None
            if self._proc is not None and (self._shape != shape or self._proc.poll() is not None):
                if self._proc.poll() is not None:
                    self.get_logger().warning(f"{self.encoder} encoder exited "
                                              f"({self._proc.returncode}); restarting")
                    self._next_start = time.monotonic() + 1.0
                self._stop_encoder()
            if self._proc is None and (time.monotonic() < self._next_start
                                       or not self._start(shape)):
                continue
            self._stamps.append((stamp, frame_id))
            try:
                self._proc.stdin.write(data)
            except (BrokenPipeError, OSError, ValueError):
                pass  # it exited: restarted on the next frame

    def _reader(self, proc: subprocess.Popen) -> None:
        fd = proc.stdout.fileno()
        buf = bytearray()
        while True:
            try:
                chunk = os.read(fd, 1 << 16)
            except OSError:
                return
            if not chunk:
                return
            buf += chunk
            for unit in split_access_units(buf, AUD["h264"]):
                self._emit(unit)

    def _emit(self, unit: bytes) -> None:
        # One access unit per input frame, in order (no B-frames): pair it with the oldest stamp.
        try:
            stamp, frame_id = self._stamps.popleft()
        except IndexError:
            stamp, frame_id = self.get_clock().now().to_msg(), self.frame_id
        msg = CompressedVideo(timestamp=TimeMsg(sec=stamp.sec, nanosec=stamp.nanosec),
                              frame_id=frame_id, format="h264", data=unit)
        self._stats["out"] += 1
        self._stats["bytes"] += len(unit)
        try:
            self.pub.publish(msg)
        except Exception:  # noqa: BLE001, S110 - the context shut down while stopping
            pass

    def _stderr(self, proc: subprocess.Popen) -> None:
        for line in iter(proc.stderr.readline, b""):
            text = line.decode(errors="replace").strip()
            if text:
                self.get_logger().warning(f"{self.encoder}: {text}", throttle_duration_sec=10.0)

    def _log_stats(self) -> None:
        s = self._stats
        dt = time.monotonic() - s["since"]
        if s["in"] or s["out"]:
            self.get_logger().info(
                f"{s['in'] / dt:.1f} fps in, {s['out'] / dt:.1f} fps out, "
                f"{s['dropped']} dropped (encoder busy), {8 * s['bytes'] / dt / 1e6:.2f} Mbit/s")
        else:
            self.get_logger().warning(
                f"no images on {self.resolve_topic_name(self.image_topic)} for {dt:.0f} s",
                throttle_duration_sec=60.0)
        self._stats = {"in": 0, "out": 0, "dropped": 0, "bytes": 0, "since": time.monotonic()}

    def destroy_node(self) -> bool:
        with self._lock:
            self._stop = True
            self._lock.notify()
        self._stop_encoder()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = ImageVideoPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
