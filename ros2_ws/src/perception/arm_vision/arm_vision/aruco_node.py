"""ArUco markers in the arm's RealSense feed -> 3D poses, TF and an annotated feed for the ground.

    ros2 launch arm_vision arm_camera.launch.py          # camera + this node (+ Foxglove bridge)
    ros2 run arm_vision aruco_node --ros-args -r __ns:=/arm_cam -p marker_size:=0.05

Subscribes (relative, i.e. /arm_cam/... in the arm_cam namespace): color/image_raw,
color/camera_info, aligned_depth_to_color/image_raw (see rgbd.py).

Publishes
  aruco/markers           arm_vision_msgs/ArucoMarkers, every processed frame (empty list = none)
  aruco/image/compressed  sensor_msgs/CompressedImage, JPEG of the colour image with each marker's
                          outline, id, axes and distance: the ground station's view (Foxglove
                          Image panel / RViz Image display). Only encoded while someone subscribes,
                          at most debug_image_rate Hz.
  aruco/visualization     visualization_msgs/MarkerArray, a plate + label per marker (3D panels)
  TF                      <colour optical frame> -> aruco_<id> for each located marker

Positions come from the aligned depth (median over the marker) when it is valid, else from
solvePnP with marker_size; orientation always from solvePnP (needs marker_size > 0). See
docs/arm-camera.md and arm_vision/aruco.py.
"""

from __future__ import annotations

import time

import cv2
import rclpy
from arm_vision_msgs.msg import ArucoMarker, ArucoMarkers
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker as VizMarker
from visualization_msgs.msg import MarkerArray

from arm_vision.aruco import ArucoDetector, draw_markers, parse_marker_sizes
from arm_vision.rgbd import RgbdFrame, RgbdSubscriber

STATS_PERIOD = 10.0  # s between status log lines


class ArucoNode(Node):

    def __init__(self):
        super().__init__("aruco_node")
        p = self.declare_parameter
        # OpenCV's thread pool fights everything else on the Jetson (ZED, OpenPose, the CSI
        # encoders, DDS) for the cores: 640x480 ArUco took 45 ms with the pool contended, ~4 ms
        # single-threaded. One thread is plenty at 30 fps; raise it for 1280x720 on a quiet CPU.
        cv2.setNumThreads(int(p("opencv_threads", 1).value))
        self.detector = ArucoDetector(
            dictionary=p("dictionary", "DICT_5X5_50").value,
            marker_size=p("marker_size", 0.05).value,
            corner_refinement=p("corner_refinement", "subpix").value,
            position_source=p("position_source", "auto").value,
            min_depth=p("min_depth", 0.1).value,
            max_depth=p("max_depth", 6.0).value,
            marker_sizes=parse_marker_sizes(p("marker_sizes", [""]).value),
        )
        self.marker_ids = set(p("marker_ids", [-1]).value) - {-1}  # [-1] = every id
        self.publish_tf = p("publish_tf", True).value
        self.tf_prefix = p("tf_prefix", "aruco_").value
        self.image_rate = p("debug_image_rate", 15.0).value
        self.image_quality = int(p("debug_image_quality", 75).value)
        self.image_width = int(p("debug_image_width", 640).value)

        self.rgbd = RgbdSubscriber(
            self, self.on_frame,
            color_topic=p("color_topic", "color/image_raw").value,
            depth_topic=p("depth_topic", "aligned_depth_to_color/image_raw").value,
            camera_info_topic=p("camera_info_topic", "color/camera_info").value,
            use_depth=p("use_depth", True).value,
            depth_sync_slop=p("depth_sync_slop", 0.02).value,
        )
        self.markers_pub = self.create_publisher(ArucoMarkers, "aruco/markers", 10)
        # Reliable, keep last 1: a reliable publisher serves best-effort AND reliable subscribers
        # (best effort refused reliable ones: "requesting incompatible QoS ... RELIABILITY", and
        # that viewer got no image). One-deep history, so a slow link can't queue stale frames.
        self.image_pub = self.create_publisher(
            CompressedImage, "aruco/image/compressed",
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.viz_pub = self.create_publisher(MarkerArray, "aruco/visualization", 10)
        self.tf = TransformBroadcaster(self) if self.publish_tf else None

        self._last_image = 0.0
        self._stats = {"frames": 0, "seconds": 0.0, "ids": set(), "since": time.monotonic()}
        self._waiting_logged = False
        self.create_timer(STATS_PERIOD, self.log_stats)
        self.get_logger().info(
            f"{self.detector.dictionary_name}, marker_size {self.detector.marker_size} m, "
            f"position {self.detector.position_source}; waiting for {self.rgbd.topics['color']}")

    # --- per frame ---------------------------------------------------------------------------

    def on_frame(self, frame: RgbdFrame):
        start = time.perf_counter()
        markers = self.detector.process(frame.color, frame.intrinsics, frame.depth)
        if self.marker_ids:
            markers = [m for m in markers if m.id in self.marker_ids]

        self.markers_pub.publish(self.to_msg(frame, markers))
        located = [m for m in markers if m.position is not None]
        if self.tf is not None and located:
            self.tf.sendTransform([self.to_tf(frame, m) for m in located])
        self.viz_pub.publish(self.to_viz(frame, located))
        self.publish_image(frame, markers)

        s = self._stats
        s["frames"] += 1
        s["seconds"] += time.perf_counter() - start
        s["ids"].update(m.id for m in markers)

    def publish_image(self, frame: RgbdFrame, markers):
        now = time.monotonic()
        if (self.image_pub.get_subscription_count() == 0 or self.image_rate <= 0
                or now - self._last_image < 1.0 / self.image_rate):
            return
        self._last_image = now
        image = draw_markers(frame.color, markers, frame.intrinsics, self.detector.marker_size,
                             self.detector.marker_sizes)
        if frame.intrinsics is None:
            cv2.putText(image, "no camera_info: 2D only", (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 0, 255), 2, cv2.LINE_AA)
        if 0 < self.image_width < image.shape[1]:
            height = round(image.shape[0] * self.image_width / image.shape[1])
            image = cv2.resize(image, (self.image_width, height), interpolation=cv2.INTER_AREA)
        ok, jpeg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, self.image_quality])
        if ok:
            self.image_pub.publish(CompressedImage(header=frame.header, format="jpeg",
                                                   data=jpeg.tobytes()))

    # --- messages ----------------------------------------------------------------------------

    def to_msg(self, frame: RgbdFrame, markers) -> ArucoMarkers:
        msg = ArucoMarkers(header=frame.header, dictionary=self.detector.dictionary_name,
                           image_width=frame.color.shape[1], image_height=frame.color.shape[0])
        for m in markers:
            msg.markers.append(ArucoMarker(
                id=m.id,
                pose=pose_of(m),
                position_source=m.position_source,
                has_orientation=m.quaternion is not None,
                distance=m.distance,
                depth=m.depth,
                reprojection_error=m.reprojection_error,
                corners=[float(v) for v in m.corners.ravel()],
            ))
        return msg

    def to_tf(self, frame: RgbdFrame, m) -> TransformStamped:
        t = TransformStamped(header=frame.header, child_frame_id=f"{self.tf_prefix}{m.id}")
        pose = pose_of(m)
        t.transform.translation = Vector3(x=pose.position.x, y=pose.position.y,
                                          z=pose.position.z)
        t.transform.rotation = pose.orientation
        return t

    def to_viz(self, frame: RgbdFrame, markers) -> MarkerArray:
        out = MarkerArray(markers=[VizMarker(header=frame.header, action=VizMarker.DELETEALL)])
        lifetime = Duration(sec=0, nanosec=500_000_000)
        for m in markers:
            size = self.detector.size_of(m.id) or 0.05
            pose = pose_of(m)
            plate = VizMarker(header=frame.header, ns="aruco", id=m.id, type=VizMarker.CUBE,
                              pose=pose, scale=Vector3(x=size, y=size, z=0.004),
                              color=ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.8), lifetime=lifetime)
            label = VizMarker(header=frame.header, ns="aruco_label", id=m.id,
                              type=VizMarker.TEXT_VIEW_FACING,
                              text=f"{m.id}: {m.distance:.2f} m",
                              scale=Vector3(z=0.03), color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0),
                              lifetime=lifetime)
            # Optical frame: y is down, so "above" the marker is -y.
            label.pose.position = Point(x=pose.position.x, y=pose.position.y - size,
                                        z=pose.position.z)
            label.pose.orientation.w = 1.0
            out.markers += [plate, label]
        return out

    # --- status ------------------------------------------------------------------------------

    def log_stats(self):
        s = self._stats
        elapsed = time.monotonic() - s["since"]
        if s["frames"] == 0:
            if not self._waiting_logged:
                self.get_logger().warning(
                    f"No images on {self.rgbd.topics['color']} for {elapsed:.0f} s. Is the "
                    "RealSense plugged in (USB 3) and realsense2_camera running? "
                    "`rs-enumerate-devices -s` lists the cameras librealsense sees.")
                self._waiting_logged = True
            s["since"] = time.monotonic()
            return
        self._waiting_logged = False
        rgbd = self.rgbd
        depth = (f"{100 * rgbd.frames_with_depth / rgbd.frames:.0f}% with depth"
                 if rgbd.topics["depth"] else "depth off")
        ids = ", ".join(str(i) for i in sorted(s["ids"])) or "none"
        self.get_logger().info(
            f"{s['frames'] / elapsed:.1f} fps, {1000 * s['seconds'] / s['frames']:.1f} ms/frame, "
            f"{depth}, {'' if rgbd.intrinsics else 'NO camera_info, '}markers seen: {ids}")
        self._stats = {"frames": 0, "seconds": 0.0, "ids": set(), "since": time.monotonic()}
        rgbd.frames = rgbd.frames_with_depth = 0


def pose_of(m) -> Pose:
    pose = Pose()
    if m.position is not None:
        x, y, z = (float(v) for v in m.position)
        pose.position = Point(x=x, y=y, z=z)
    if m.quaternion is not None:
        qx, qy, qz, qw = (float(v) for v in m.quaternion)
        pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    else:
        pose.orientation.w = 1.0
    return pose


def main():
    rclpy.init()
    try:
        rclpy.spin(ArucoNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
