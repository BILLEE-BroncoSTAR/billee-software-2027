#!/usr/bin/env python3
"""One CSI camera (Argus) -> hardware H.264 + JPEG -> ROS, for Foxglove and RViz.

    ros2 run camera_stream csi_compressed_video_pub --ros-args -p sensor_id:=0 -p name:=front \\
        -p video_topic:=/cam0/video -p jpeg_topic:=/cam0/image_raw/compressed

Publishes
  video_topic  foxglove_msgs/CompressedVideo, H.264 (Annex B), one frame per message:
               Foxglove's Image panel. About `bitrate` bits/s whatever the scene.
  jpeg_topic   sensor_msgs/CompressedImage, JPEG at jpeg_width x jpeg_height, jpeg_fps: RViz's
               Image display (a topic ending in /compressed selects that transport) and Foxglove.
               DDS sends it only while something subscribes.

Why a gst-launch-1.0 child process instead of GStreamer's Python bindings: ROS here comes from the
pixi environment (Python 3.12), which has no GStreamer bindings, while NVIDIA's camera and encoder
plugins (nvarguscamerasrc, nvv4l2h264enc, nvjpegenc) are built for the system GStreamer. The
system gst-launch-1.0 runs the pipeline in its own process with the system libraries and writes
the H.264 byte stream to stdout and the JPEGs to a second pipe; this node splits them into frames.

A missing camera never crashes the launch. If the sensor wasn't detected at boot, or the Argus
socket or gst-launch-1.0 is missing, or the pipeline fails, the node logs one warning, publishes a
low-rate placeholder on both topics ("NO CAMERA" text when the pango plugin is installed, colour
bars otherwise), and checks again every recheck_s seconds. source:=test sends a moving test
pattern through the same encoders, to test with no camera.
"""

from __future__ import annotations

import collections
import glob
import os
import signal
import subprocess
import threading
import time
from typing import Callable, List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

try:
    from foxglove_msgs.msg import CompressedVideo
except ImportError:  # warned about in __init__
    CompressedVideo = None

GST_PLUGIN_DIR = "/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
ARGUS_SOCKET = "/tmp/argus_socket"
# nvarguscamerasrc, nvvidconv, nvv4l2h26xenc, nvjpegenc
NVIDIA_PLUGINS = ("nvarguscamerasrc", "nvvidconv", "nvvideo4linux2", "nvjpeg")

# Access unit delimiter NAL unit after a 3-byte start code. insert-aud=true makes the encoder put
# one at the start of every frame, so the byte stream splits into frames there. 00 00 01 can't
# occur inside a NAL unit (emulation prevention), so the pattern only matches real delimiters.
AUD = {"h264": b"\x00\x00\x01\x09", "h265": b"\x00\x00\x01\x46\x01"}


def split_access_units(buf: bytearray, aud: bytes) -> List[bytes]:
    """Remove and return the complete access units at the front of buf.

    A unit runs from one delimiter to the next, so the newest stays in buf until the next frame
    starts. Bytes before the first delimiter (a stream joined mid-frame) are dropped.
    """
    starts = []
    i = buf.find(aud)
    while i != -1:
        starts.append(i - 1 if i > 0 and buf[i - 1] == 0 else i)  # 4-byte start code
        i = buf.find(aud, i + len(aud))
    if len(starts) < 2:
        if starts and starts[0] > 0:
            del buf[: starts[0]]
        elif not starts and len(buf) > len(aud):
            del buf[: -len(aud)]  # keep a start code + delimiter split across reads
        return []
    units = [bytes(buf[a:b]) for a, b in zip(starts, starts[1:])]
    del buf[: starts[-1]]
    return units


def split_jpegs(buf: bytearray) -> List[bytes]:
    """Remove and return the complete JPEGs (SOI ... EOI) at the front of buf.

    0xFF inside JPEG entropy-coded data is stuffed (FF 00), so FF D9 only appears as the end.
    """
    images = []
    while True:
        soi = buf.find(b"\xff\xd8")
        if soi == -1:
            if len(buf) > 1:
                del buf[:-1]  # keep an FF that may start a marker split across reads
            return images
        eoi = buf.find(b"\xff\xd9", soi + 2)
        if eoi == -1:
            del buf[:soi]
            return images
        images.append(bytes(buf[soi: eoi + 2]))
        del buf[: eoi + 2]


def detected_csi_sensors() -> int:
    """CSI sensors the kernel probed at boot. Argus numbers them 0..n-1 in device-tree order."""
    count = 0
    for path in glob.glob("/sys/class/video4linux/video*/name"):
        try:
            with open(path) as fh:
                count += fh.read().startswith("vi-output")
        except OSError:
            pass
    return count


def have_plugin(name: str) -> bool:
    return os.path.exists(os.path.join(GST_PLUGIN_DIR, f"libgst{name}.so"))


class CSICompressedVideoPublisher(Node):
    def __init__(self) -> None:
        super().__init__("csi_compressed_video_publisher")

        p = self.declare_parameter
        self.cam_name = str(p("name", "camera").value)
        self.sensor_id = int(p("sensor_id", 0).value)
        self.source = str(p("source", "argus").value)  # argus | test
        self.video_topic = str(p("video_topic", "/cam0/video").value)
        self.jpeg_topic = str(p("jpeg_topic", "/cam0/image_raw/compressed").value)
        self.frame_id = str(p("frame_id", "cam0_optical").value)
        self.width = int(p("width", 1280).value)
        self.height = int(p("height", 720).value)
        self.fps = int(p("fps", 30).value)
        self.encoding = str(p("encoding", "h264").value).lower()
        self.bitrate = int(p("bitrate", 3_000_000).value)
        self.gop = int(p("gop", 30).value)
        self.flip_method = int(p("flip_method", 0).value)
        self.jpeg_width = int(p("jpeg_width", 640).value)
        self.jpeg_height = int(p("jpeg_height", 360).value)
        self.jpeg_fps = int(p("jpeg_fps", 10).value)  # 0 = no JPEG topic
        self.jpeg_quality = int(p("jpeg_quality", 70).value)
        self.placeholder_fps = max(1, int(p("placeholder_fps", 2).value))
        self.recheck_s = float(p("recheck_s", 10.0).value)
        self.gst_launch = str(p("gst_launch", "/usr/bin/gst-launch-1.0").value)

        if self.encoding not in AUD:
            raise ValueError(f"encoding must be h264 or h265, not {self.encoding!r}")
        if self.source not in ("argus", "test"):
            raise ValueError(f"source must be argus or test, not {self.source!r}")
        if not 0 <= self.flip_method <= 7:
            raise ValueError(f"flip_method must be 0-7 (nvvidconv), not {self.flip_method}")
        self.label = f"cam{self.sensor_id} {self.cam_name}"

        # Best effort, newest frames only: a slow link drops frames instead of building latency.
        # Subscribers must be best effort too (RViz: Reliability Policy "Best Effort").
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.video_pub = None
        if CompressedVideo is not None:
            self.video_pub = self.create_publisher(CompressedVideo, self.video_topic, qos)
        else:
            self.get_logger().warning(
                f"{self.label}: foxglove_msgs isn't built, so no H.264 on {self.video_topic}, "
                f"only JPEG on {self.jpeg_topic}. Build it: "
                "pixi run colcon build --packages-select foxglove_msgs camera_stream")
        self.jpeg_pub = None
        if self.jpeg_fps > 0:
            self.jpeg_pub = self.create_publisher(CompressedImage, self.jpeg_topic, qos)

        self._stop = threading.Event()
        self._proc: Optional[subprocess.Popen] = None
        self._proc_lock = threading.Lock()
        self._frames = 0
        self._supervisor = threading.Thread(target=self._supervise, daemon=True)
        self._supervisor.start()

    # ---- what can run ----

    def _encoder_missing(self) -> Optional[str]:
        """Why nothing at all can be encoded here (no placeholder either), or None."""
        if not os.path.exists(self.gst_launch):
            return (f"{self.gst_launch} is missing: rebuild the Jetson dev container "
                    "(it installs gstreamer1.0-tools)")
        missing = [name for name in NVIDIA_PLUGINS if not have_plugin(name)]
        if missing:
            return f"no NVIDIA GStreamer plugins ({', '.join(missing)}): not a Jetson container"
        return None

    def _camera_missing(self) -> Optional[str]:
        """Why the camera itself can't stream now, or None."""
        reason = self._encoder_missing()
        if reason or self.source == "test":
            return reason
        if not os.path.exists(ARGUS_SOCKET):
            return (f"no Argus socket ({ARGUS_SOCKET}) in this container: rebuild the Jetson "
                    "dev container (it mounts it)")
        found = detected_csi_sensors()
        if self.sensor_id >= found:
            return (f"no camera on sensor-id {self.sensor_id} ({found} CSI camera(s) detected at "
                    "boot): connect it with the Jetson powered off, then boot")
        return None

    def _output_size(self):
        """(width, height, jpeg_width, jpeg_height) after flip_method.

        flip_method is nvvidconv's: 0 none, 2 rotate 180, 4 mirror left-right, 6 mirror
        top-bottom; 1/3 rotate 90 counter-/clockwise and 5/7 transpose, which swap width and
        height (a portrait stream).
        """
        if self.flip_method in (1, 3, 5, 7):
            return self.height, self.width, self.jpeg_height, self.jpeg_width
        return self.width, self.height, self.jpeg_width, self.jpeg_height

    def _pipeline(self, placeholder: bool, jpeg_fd: Optional[int]) -> List[str]:
        """gst-launch-1.0 argv: source -> tee -> H.264 to fd 1, JPEG to jpeg_fd."""
        out_w, out_h, jpeg_w, jpeg_h = self._output_size()
        nvmm_out = f"video/x-raw(memory:NVMM),width={out_w},height={out_h}"
        gop = self.gop
        if placeholder or self.source == "test":
            # Same size as the camera's stream, so a viewer's decoder needn't reconfigure.
            rate = self.placeholder_fps if placeholder else self.fps
            text = placeholder and have_plugin("pango")
            pattern = ("black" if text else "smpte") if placeholder else "ball"
            src = ["videotestsrc", "is-live=true", f"pattern={pattern}", "!",
                   f"video/x-raw,format=I420,width={out_w},height={out_h},framerate={rate}/1"]
            if text:
                src += ["!", "textoverlay", f"text={self.label}: NO CAMERA", "valignment=center",
                        "halignment=center", "font-desc=Sans 36"]
            src += ["!", "nvvidconv", "!", f"{nvmm_out},format=NV12"]
            if placeholder:
                gop = rate  # a keyframe every second, so a new viewer sees it at once
        else:
            src = ["nvarguscamerasrc", f"sensor-id={self.sensor_id}", "!",
                   f"video/x-raw(memory:NVMM),width={self.width},height={self.height},"
                   f"framerate={self.fps}/1"]
            if self.flip_method:
                # In the Jetson's video hardware (VIC), before encoding: every viewer gets it.
                src += ["!", "nvvidconv", f"flip-method={self.flip_method}", "!", nvmm_out]
        enc = "nvv4l2h264enc" if self.encoding == "h264" else "nvv4l2h265enc"
        argv = src + [
            "!", "tee", "name=t",
            "t.", "!", "queue", "max-size-buffers=2", "leaky=downstream",
            "!", enc, f"bitrate={self.bitrate}", "control-rate=1", f"iframeinterval={gop}",
            f"idrinterval={gop}", "insert-sps-pps=true", "insert-aud=true", "num-B-Frames=0",
            "!", f"video/x-{self.encoding},stream-format=byte-stream",
            "!", "fdsink", "fd=1", "sync=false",
        ]
        if jpeg_fd is not None:
            argv += [
                "t.", "!", "queue", "max-size-buffers=1", "leaky=downstream",
                "!", "nvvidconv",
                "!", f"video/x-raw,width={jpeg_w},height={jpeg_h},format=I420",
                "!", "videorate", "drop-only=true", f"max-rate={self.jpeg_fps}",
                "!", "nvjpegenc", f"quality={self.jpeg_quality}",
                "!", "fdsink", f"fd={jpeg_fd}", "sync=false",
            ]
        return argv

    # ---- supervision ----

    def _supervise(self) -> None:
        warned = None  # the kind of problem last warned about, to warn once per change
        backoff = self.recheck_s
        while not self._stop.is_set():
            reason = self._camera_missing()
            kind = reason
            if reason is None:
                if warned:
                    self.get_logger().info(f"{self.label}: starting the camera")
                self._frames = 0
                started = time.monotonic()
                err = self._run(placeholder=False, stop_when=None)
                if self._stop.is_set():
                    return
                if self._frames:
                    kind = "stopped"
                    reason = (f"camera stopped after {time.monotonic() - started:.0f} s "
                              f"({err or 'no error output'})")
                    backoff = self.recheck_s
                else:
                    kind = "failed"
                    reason = f"camera failed to start ({err or 'no error output'})"
                    backoff = min(backoff * 2, 120.0) if warned == "failed" else self.recheck_s
            else:
                backoff = self.recheck_s
            if kind != warned:
                self.get_logger().warning(
                    f"{self.label}: {reason}. Publishing a placeholder on {self.video_topic}; "
                    f"checking again every {self.recheck_s:.0f} s.")
                warned = kind
            if self._encoder_missing():
                self._stop.wait(backoff)  # nothing can be encoded, not even the placeholder
                continue
            retry_at = time.monotonic() + backoff
            self._run(placeholder=True,
                      stop_when=lambda: (time.monotonic() >= retry_at
                                         and self._camera_missing() is None))

    def _run(self, placeholder: bool, stop_when: Optional[Callable[[], bool]]) -> str:
        """Run one gst-launch-1.0 until it exits, stop_when() is true, or shutdown.

        Returns its last line of error output.
        """
        jpeg_r = jpeg_w = None
        if self.jpeg_pub is not None:
            jpeg_r, jpeg_w = os.pipe()
        argv = self._pipeline(placeholder, jpeg_w)
        # A clean environment: the system GStreamer, nothing from the pixi environment.
        env = {k: v for k, v in os.environ.items()
               if k in ("HOME", "USER", "LANG", "LC_ALL", "XDG_RUNTIME_DIR")}
        env["PATH"] = "/usr/bin:/bin"
        # setpriv --pdeathsig: gst-launch-1.0 dies with this node, even after a kill -9.
        prefix = (["/usr/bin/setpriv", "--pdeathsig", "TERM", "--"]
                  if os.path.exists("/usr/bin/setpriv") else [])
        stderr_tail: collections.deque = collections.deque(maxlen=20)
        try:
            proc = subprocess.Popen(prefix + [self.gst_launch, "-q", "-e"] + argv,
                                    stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, env=env,
                                    pass_fds=(jpeg_w,) if jpeg_w is not None else ())
        except OSError as exc:
            if jpeg_r is not None:
                os.close(jpeg_r)
            return str(exc)
        finally:
            if jpeg_w is not None:
                os.close(jpeg_w)  # exactly once; the child has its own copy
        with self._proc_lock:
            self._proc = proc
        readers = [threading.Thread(target=self._read_video, args=(proc.stdout,), daemon=True),
                   threading.Thread(target=self._read_lines, args=(proc.stderr, stderr_tail),
                                    daemon=True)]
        if jpeg_r is not None:
            readers.append(threading.Thread(target=self._read_jpeg, args=(jpeg_r,), daemon=True))
        for t in readers:
            t.start()
        if not placeholder:
            out_w, out_h, jpeg_w, jpeg_h = self._output_size()
            self.get_logger().info(
                f"{self.label}: {self.source} sensor-id={self.sensor_id}"
                + (f" flip-method={self.flip_method}" if self.flip_method else "")
                + f" -> {self.encoding} {out_w}x{out_h}@{self.fps} {self.bitrate // 1000} kbit/s "
                f"on {self.video_topic}"
                + (f", JPEG {jpeg_w}x{jpeg_h}@{self.jpeg_fps} on {self.jpeg_topic}"
                   if self.jpeg_pub else ""))
        next_check = time.monotonic() + 1.0
        while proc.poll() is None and not self._stop.is_set():
            if stop_when is not None and time.monotonic() >= next_check:
                if stop_when():
                    break
                next_check = time.monotonic() + 1.0
            time.sleep(0.2)
        self._terminate(proc)
        for t in readers:
            t.join(timeout=2.0)
        if jpeg_r is not None:
            try:
                os.close(jpeg_r)
            except OSError:
                pass
        with self._proc_lock:
            self._proc = None
        errors = [line.strip() for line in stderr_tail if line.strip()]
        return errors[-1] if errors else ""

    @staticmethod
    def _terminate(proc: subprocess.Popen) -> None:
        if proc.poll() is not None:
            return
        # SIGINT with -e: gst-launch-1.0 sends EOS and closes the camera cleanly. Killing it
        # outright can leave the Argus daemon needing a restart.
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()

    # ---- readers (one thread each) ----

    def _publish(self, pub, msg) -> bool:
        try:
            pub.publish(msg)
            return True
        except Exception:  # context shut down while stopping
            return False

    def _read_video(self, stream) -> None:
        fd = stream.fileno()
        buf = bytearray()
        aud = AUD[self.encoding]
        while chunk := self._read(fd):
            buf += chunk
            for unit in split_access_units(buf, aud):
                self._frames += 1
                if self.video_pub is None:
                    continue
                msg = CompressedVideo()
                msg.timestamp = self.get_clock().now().to_msg()
                msg.frame_id = self.frame_id
                msg.format = self.encoding
                msg.data = unit
                if not self._publish(self.video_pub, msg):
                    return

    def _read_jpeg(self, fd: int) -> None:
        buf = bytearray()
        while chunk := self._read(fd):
            buf += chunk
            for jpeg in split_jpegs(buf):
                msg = CompressedImage()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.format = "jpeg"
                msg.data = jpeg
                if not self._publish(self.jpeg_pub, msg):
                    return

    @staticmethod
    def _read(fd: int) -> bytes:
        try:
            return os.read(fd, 1 << 16)
        except OSError:
            return b""

    def _read_lines(self, stream, sink: collections.deque) -> None:
        for raw in iter(stream.readline, b""):
            line = raw.decode(errors="replace").rstrip()
            sink.append(line)
            self.get_logger().debug(f"{self.label} gst: {line}")

    def destroy_node(self) -> bool:
        self._stop.set()
        with self._proc_lock:
            proc = self._proc
        if proc is not None:
            self._terminate(proc)
        self._supervisor.join(timeout=5.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = CSICompressedVideoPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ctrl-C under ros2 launch arrives twice (the terminal signals the process group, then
        # launch signals each node). Ignore the second so it can't cut short the wait that lets
        # gst-launch-1.0 close the camera cleanly.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
