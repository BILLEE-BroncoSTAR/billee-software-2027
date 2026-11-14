#!/usr/bin/env python3
"""
CSI (Argus) camera -> NVENC -> foxglove_msgs/CompressedVideo publisher.

Launch example (one camera):
  ros2 run <your_pkg> csi_compressed_video_pub --ros-args \
    -p sensor_id:=0 \
    -p width:=1280 -p height:=720 -p fps:=30 \
    -p bitrate:=3000000 -p gop:=30 \
    -p encoding:=h264 \
    -p topic:=/cam0/video -p frame_id:=cam0_optical

Run 6 cameras by launching 6 instances with different sensor_id/topic/frame_id.
"""

from __future__ import annotations

import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from foxglove_msgs.msg import CompressedVideo
from sensor_msgs.msg import CompressedImage

import gi  # type: ignore
gi.require_version("Gst", "1.0")
from gi.repository import Gst, GLib  # type: ignore


class CSICompressedVideoPublisher(Node):
    def __init__(self) -> None:
        super().__init__("csi_compressed_video_publisher")

        # ---- Parameters ----
        self.declare_parameter("topic", "/camera/video")
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("sensor_id", 0)

        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 720)
        self.declare_parameter("fps", 30)

        self.declare_parameter("encoding", "h264")          # "h264" or "h265"
        self.declare_parameter("bitrate", 3_000_000)        # bits/sec
        self.declare_parameter("gop", 30)                   # frames between keyframes/IDRs

        # If True, use GStreamer PTS (anchored) instead of ROS "now"
        self.declare_parameter("use_pts", False)

        # Optional: "flip-method" for nvarguscamerasrc (0..7)
        self.declare_parameter("flip_method", 0)

        self.topic: str = str(self.get_parameter("topic").value)
        self.frame_id: str = str(self.get_parameter("frame_id").value)
        self.sensor_id: int = int(self.get_parameter("sensor_id").value)

        self.width: int = int(self.get_parameter("width").value)
        self.height: int = int(self.get_parameter("height").value)
        self.fps: int = int(self.get_parameter("fps").value)

        self.encoding: str = str(self.get_parameter("encoding").value).lower()
        self.bitrate: int = int(self.get_parameter("bitrate").value)
        self.gop: int = int(self.get_parameter("gop").value)

        self.use_pts: bool = bool(self.get_parameter("use_pts").value)
        self.flip_method: int = int(self.get_parameter("flip_method").value)

        if self.encoding not in ("h264", "h265"):
            raise ValueError("encoding must be 'h264' or 'h265'")

        # ---- QoS: best-effort video (drop frames, don't build latency) ----
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(CompressedVideo, self.topic, qos)
        self.pub_image = self.create_publisher(CompressedImage, self.topic + "/image", qos)

        # ---- GStreamer ----
        Gst.init(None)

        self._pipeline: Optional[Gst.Pipeline] = None
        self._appsink: Optional[Gst.Element] = None
        self._appsink_image: Optional[Gst.Element] = None

        # GLib loop for appsink callbacks
        self._glib_loop = GLib.MainLoop()
        self._glib_thread = threading.Thread(target=self._glib_loop.run, daemon=True)

        self._ros_anchor_ns: Optional[int] = None  # for PTS->ROS stamping

        self._build_and_start_pipeline()
        self._glib_thread.start()

        self.get_logger().info(
            f"CSI sensor-id={self.sensor_id} -> {self.encoding} -> {self.topic} "
            f"({self.width}x{self.height}@{self.fps}, bitrate={self.bitrate}, gop={self.gop})"
        )

    def _pipeline_string(self) -> str:
        # Capture
        # nvarguscamerasrc outputs NVMM buffers (GPU-friendly)
        src = (
            f"nvarguscamerasrc sensor-id={self.sensor_id} ! "
            f"video/x-raw(memory:NVMM),width={self.width},height={self.height},framerate={self.fps}/1 ! "
        )

        # Optional flip
        if self.flip_method != 0:
            src += f"nvvidconv flip-method={self.flip_method} ! video/x-raw(memory:NVMM),width={self.width},height={self.height},framerate={self.fps}/1 ! "

        # Encode + parse
        # Foxglove CompressedVideo constraints:
        # - no B-frames
        # - Annex-B byte-stream
        # - access-unit alignment (one frame per buffer/message)
        if self.encoding == "h264":
            enc = (
                "nvv4l2h264enc "
                f"bitrate={self.bitrate} "
                "insert-sps-pps=1 "
                f"iframeinterval={self.gop} idrinterval={self.gop} "
                "num-B-Frames=0 ! "
                "h264parse config-interval=1 ! "
                "video/x-h264,stream-format=byte-stream,alignment=au ! "
            )
        else:
            enc = (
                "nvv4l2h265enc "
                f"bitrate={self.bitrate} "
                "insert-sps-pps=1 "
                f"iframeinterval={self.gop} idrinterval={self.gop} "
                "num-B-Frames=0 ! "
                "h265parse config-interval=1 ! "
                "video/x-h265,stream-format=byte-stream,alignment=au ! "
            )

        video_sink = "appsink name=vsink emit-signals=true sync=false max-buffers=1 drop=true"
        image_sink = "appsink name=jsink emit-signals=true sync=false max-buffers=1 drop=true"

        jpeg_branch = (
            "queue ! "
            "nvvidconv ! video/x-raw,format=I420 ! "
            "nvjpegenc ! image/jpeg ! "
        )

        return (
            src
            + "tee name=t "
            + "t. ! queue ! "
            + enc
            + video_sink
            + " t. ! "
            + jpeg_branch
            + image_sink
        )

    def _build_and_start_pipeline(self) -> None:
        pipe_str = self._pipeline_string()
        self.get_logger().info(f"GStreamer pipeline:\n{pipe_str}")

        pipeline = Gst.parse_launch(pipe_str)
        if not isinstance(pipeline, Gst.Pipeline):
            raise RuntimeError("Failed to create GStreamer pipeline")

        appsink = pipeline.get_by_name("vsink")
        if appsink is None:
            raise RuntimeError("Failed to find appsink 'vsink'")

        appsink_image = pipeline.get_by_name("jsink")
        if appsink_image is None:
            raise RuntimeError("Failed to find appsink 'jsink'")

        appsink.connect("new-sample", self._on_new_sample_video)
        appsink_image.connect("new-sample", self._on_new_sample_image)

        self._pipeline = pipeline
        self._appsink = appsink
        self._appsink_image = appsink_image
        self._ros_anchor_ns = self.get_clock().now().nanoseconds

        ret = self._pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError("Failed to set pipeline to PLAYING")

    def _stamp_from_pts(self, pts_ns: int) -> rclpy.time.Time:
        anchor = self._ros_anchor_ns if self._ros_anchor_ns is not None else self.get_clock().now().nanoseconds
        return rclpy.time.Time(nanoseconds=anchor + int(pts_ns))

    def _on_new_sample_video(self, sink: Gst.Element) -> Gst.FlowReturn:
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf: Gst.Buffer = sample.get_buffer()
        if buf is None:
            return Gst.FlowReturn.ERROR

        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR

        try:
            payload = bytes(mapinfo.data)  # one access unit (one frame) due to alignment=au
        finally:
            buf.unmap(mapinfo)

        msg = CompressedVideo()
        msg.format = self.encoding
        msg.data = payload
        msg.frame_id = self.frame_id

        if self.use_pts and buf.pts != Gst.CLOCK_TIME_NONE:
            t = self._stamp_from_pts(int(buf.pts))
            msg.timestamp.sec = int(t.nanoseconds // 1_000_000_000)
            msg.timestamp.nanosec = int(t.nanoseconds % 1_000_000_000)
        else:
            msg.timestamp = self.get_clock().now().to_msg()

        self.pub.publish(msg)
        return Gst.FlowReturn.OK

    def _on_new_sample_image(self, sink: Gst.Element) -> Gst.FlowReturn:
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR

        buf: Gst.Buffer = sample.get_buffer()
        if buf is None:
            return Gst.FlowReturn.ERROR

        ok, mapinfo = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR

        try:
            payload = bytes(mapinfo.data)
        finally:
            buf.unmap(mapinfo)

        img_msg = CompressedImage()
        img_msg.format = "jpeg"
        img_msg.data = payload
        img_msg.header.frame_id = self.frame_id

        if self.use_pts and buf.pts != Gst.CLOCK_TIME_NONE:
            t = self._stamp_from_pts(int(buf.pts))
            img_msg.header.stamp.sec = int(t.nanoseconds // 1_000_000_000)
            img_msg.header.stamp.nanosec = int(t.nanoseconds % 1_000_000_000)
        else:
            img_msg.header.stamp = self.get_clock().now().to_msg()

        self.pub_image.publish(img_msg)
        return Gst.FlowReturn.OK

    def destroy_node(self) -> bool:
        try:
            if self._pipeline is not None:
                self._pipeline.set_state(Gst.State.NULL)
        finally:
            if self._glib_loop.is_running():
                self._glib_loop.quit()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = CSICompressedVideoPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
