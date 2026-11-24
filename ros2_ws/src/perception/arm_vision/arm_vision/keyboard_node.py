"""Keyboard located from ArUco markers -> its pose and every key's position relative to the rover.

    ros2 launch arm_vision arm_camera.launch.py keyboard:=true      # camera + aruco + this node
    ros2 run arm_vision keyboard_node --ros-args -r __ns:=/arm_cam -p target_frame:=base_link

Subscribes (relative: /arm_cam/...): aruco/markers (aruco_node), color/camera_info.

Publishes
  keyboard/keys           arm_vision_msgs/Keyboard: pose + every key's top centre, in target_frame
  keyboard/pose           geometry_msgs/PoseStamped, the keyboard frame in target_frame
  keyboard/visualization  visualization_msgs/MarkerArray: outline, keycaps, labels, in the keyboard
                          frame (Foxglove / RViz 3D, and Foxglove's Image panel draws them over
                          the camera image)
  TF                      target_frame -> keyboard

Service: ~/reset (std_srvs/Trigger) forgets the held estimate (after the rover moved).

How: every aruco/markers message with layout markers in it gives one pose (arm_vision/keyboard.py:
multi-marker solvePnP, translation corrected by the markers' depth). It goes through TF to
target_frame at the image's stamp (base_link: through the arm's joint angles and the camera
mount, arm_description), is smoothed there, and is held while no marker is in view, so the arm
keeps the key positions when it gets too close to see the markers. The keyboard is assumed still
relative to target_frame: reset (or let a new detection jump it) once the rover moves.
"""

from __future__ import annotations

import os

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from arm_vision_msgs.msg import ArucoMarkers, Keyboard, KeyboardKey
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    TransformStamped,
    Vector3,
)
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from arm_vision.aruco import Intrinsics, parse_marker_sizes, rotation_to_quaternion
from arm_vision.keyboard import (
    KeyboardLayout,
    KeyboardLocalizer,
    MarkerObservation,
    PoseFilter,
    quaternion_to_rotation,
)

STATUS_PERIOD = 10.0  # s


class KeyboardNode(Node):

    def __init__(self):
        super().__init__("keyboard_node")
        p = self.declare_parameter
        layout_file = p("layout_file", "").value or os.path.join(
            get_package_share_directory("arm_vision"), "config", "keyboard_layout.yaml")
        self.layout = KeyboardLayout.from_yaml(layout_file)
        # Printed sizes set at launch (marker_sizes:=, the same list aruco_node gets) replace the
        # layout's for those ids: PnP over the layout's corners needs the real size.
        size_override = parse_marker_sizes(p("marker_sizes", [""]).value)
        if size_override:
            self.layout = self.layout.with_marker_sizes(size_override)
        self.localizer = KeyboardLocalizer(
            self.layout,
            min_markers=p("min_markers", 1).value,
            max_reprojection_error=p("max_reprojection_error", 4.0).value,
            use_depth=p("use_depth", True).value,
            max_depth_correction=p("max_depth_correction", 0.05).value,
        )
        self.target_frame = p("target_frame", "base_link").value
        self.keyboard_frame = p("keyboard_frame", "keyboard").value
        self.hold_timeout = float(p("hold_timeout", 0.0).value)  # s; 0 = until reset
        self.tf_timeout = Duration(seconds=float(p("tf_timeout", 0.1).value))
        self.filter = PoseFilter(alpha=p("smoothing", 0.3).value,
                                 reset_distance=p("reset_distance", 0.05).value,
                                 reset_angle=np.radians(p("reset_angle", 10.0).value))
        publish_rate = float(p("publish_rate", 10.0).value)

        self.intrinsics: Intrinsics | None = None
        self.frame_id = ""          # frame the held pose is in
        self.last_seen: Time | None = None
        self.last_estimate = None   # KeyboardPose
        self._stats = {"messages": 0, "estimates": 0, "rejected": {}, "tf_failed": 0}

        self.tf_buffer = Buffer()
        # main() spins this node in a MultiThreadedExecutor: the listener's subscriptions are in
        # their own reentrant callback group, so TF keeps arriving while on_markers waits in
        # lookup_transform, and this node's own callbacks (default group) never run at once.
        # Not spin_thread=True: in Humble that spins the whole node in a second executor too
        # (on_markers and the timers racing on the filter), and its thread prints an
        # ExternalShutdownException traceback on every Ctrl-C.
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf = TransformBroadcaster(self)
        self.keys_pub = self.create_publisher(Keyboard, "keyboard/keys", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "keyboard/pose", 10)
        self.viz_pub = self.create_publisher(MarkerArray, "keyboard/visualization", 10)
        self.create_subscription(CameraInfo, p("camera_info_topic", "color/camera_info").value,
                                 self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(ArucoMarkers, p("markers_topic", "aruco/markers").value,
                                 self.on_markers, 10)
        self.create_service(Trigger, "~/reset", self.on_reset)
        if publish_rate > 0:
            self.create_timer(1.0 / publish_rate, self.publish_held)
        self.create_timer(1.0, self.publish_viz)
        self.create_timer(STATUS_PERIOD, self.log_status)
        ids = ", ".join(f"{i} ({1000 * m.size:g} mm)"
                        for i, m in sorted(self.layout.markers.items()))
        self.get_logger().info(
            f"layout {self.layout.name!r} ({layout_file}): markers {ids}, "
            f"{len(self.layout.keys)} keys; keyboard in {self.target_frame or 'the camera frame'}")

    # --- inputs ------------------------------------------------------------------------------

    def on_camera_info(self, msg: CameraInfo):
        intrinsics = Intrinsics.from_camera_info(msg.k, msg.d)
        if intrinsics.valid:
            self.intrinsics = intrinsics

    def on_markers(self, msg: ArucoMarkers):
        self._stats["messages"] += 1
        observations = [
            MarkerObservation(
                id=m.id, corners=np.asarray(m.corners, dtype=float).reshape(4, 2),
                depth_position=(np.array([m.pose.position.x, m.pose.position.y,
                                          m.pose.position.z])
                                if m.position_source == "depth" else None))
            for m in msg.markers if m.id in self.layout.markers]
        if not observations:
            return
        if self.intrinsics is None:
            self._reject("no camera_info yet")
            return
        estimate = self.localizer.estimate(observations, self.intrinsics)
        if estimate is None:
            self._reject(self.localizer.last_rejection)
            return

        T = estimate.matrix()  # keyboard -> camera
        frame = msg.header.frame_id
        if self.target_frame and self.target_frame != frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.target_frame, frame,
                                                     Time.from_msg(msg.header.stamp),
                                                     self.tf_timeout)
            except TransformException as e:
                self._stats["tf_failed"] += 1
                self.get_logger().warning(
                    f"keyboard seen but no TF {self.target_frame} <- {frame}: {e}. Is the arm's "
                    "robot_state_publisher running (arm_description.launch.py) with "
                    "/arm/joint_states?", throttle_duration_sec=5.0)
                return
            T = transform_matrix(tf.transform) @ T
            frame = self.target_frame
        if frame != self.frame_id:
            self.filter.reset()
            self.frame_id = frame
        position, quaternion = self.filter.update(T[:3, 3], rotation_to_quaternion(T[:3, :3]))
        self.last_estimate = estimate
        self.last_seen = Time.from_msg(msg.header.stamp)
        self._stats["estimates"] += 1
        self.publish(msg.header.stamp, position, quaternion, tracking=True)

    def on_reset(self, _request, response):
        self.filter.reset()
        self.last_estimate = self.last_seen = None
        response.success = True
        response.message = "keyboard estimate cleared"
        self.get_logger().info(response.message)
        return response

    def _reject(self, reason: str):
        r = self._stats["rejected"]
        r[reason] = r.get(reason, 0) + 1

    # --- outputs -----------------------------------------------------------------------------

    def publish_held(self):
        if self.filter.position is None or self.last_seen is None:
            return
        now = self.get_clock().now()
        age = (now - self.last_seen).nanoseconds * 1e-9
        if age < 0.2:  # on_markers just published it
            return
        if self.hold_timeout > 0 and age > self.hold_timeout:
            self.get_logger().info(f"keyboard not seen for {age:.0f} s: estimate dropped")
            self.filter.reset()
            self.last_seen = None
            return
        self.publish(now.to_msg(), self.filter.position, self.filter.quaternion, tracking=False)

    def publish(self, stamp, position, quaternion, tracking: bool):
        pose = Pose(position=Point(x=float(position[0]), y=float(position[1]),
                                   z=float(position[2])),
                    orientation=Quaternion(x=float(quaternion[0]), y=float(quaternion[1]),
                                           z=float(quaternion[2]), w=float(quaternion[3])))
        t = TransformStamped(child_frame_id=self.keyboard_frame)
        t.header.stamp, t.header.frame_id = stamp, self.frame_id
        t.transform.translation = Vector3(x=pose.position.x, y=pose.position.y,
                                          z=pose.position.z)
        t.transform.rotation = pose.orientation
        self.tf.sendTransform(t)

        ps = PoseStamped(pose=pose)
        ps.header = t.header
        self.pose_pub.publish(ps)

        R = quaternion_to_rotation(quaternion)
        est = self.last_estimate
        msg = Keyboard(header=t.header, layout=self.layout.name, pose=pose, tracking=tracking,
                       last_seen=self.last_seen.to_msg() if self.last_seen else stamp,
                       marker_ids=est.marker_ids if est else [],
                       reprojection_error=est.reprojection_error if est else float("nan"),
                       depth_corrected=bool(est is not None and est.depth_correction is not None))
        for key in self.layout.keys.values():
            x, y, z = R @ key.center + position
            msg.keys.append(KeyboardKey(label=key.label, position=Point(x=x, y=y, z=z),
                                        width=key.width, height=key.height))
        self.keys_pub.publish(msg)

    def publish_viz(self):
        if self.filter.position is None:
            return
        self.viz_pub.publish(keyboard_markers(self.layout, self.keyboard_frame,
                                              self.get_clock().now().to_msg()))

    def log_status(self):
        s = self._stats
        if s["messages"] == 0:
            return
        parts = [f"{s['estimates']} estimates from {s['messages']} marker messages"]
        if self.last_estimate is not None:
            e = self.last_estimate
            parts.append(f"last: markers {e.marker_ids}, {e.reprojection_error:.2f} px, "
                         f"{'depth-corrected' if e.depth_correction is not None else 'PnP only'}")
        if self.filter.position is not None:
            parts.append("keyboard at " + ", ".join(f"{v:.3f}" for v in self.filter.position)
                         + f" m in {self.frame_id}")
        if s["tf_failed"]:
            parts.append(f"{s['tf_failed']} without TF to {self.target_frame}")
        for reason, n in s["rejected"].items():
            parts.append(f"rejected {n}x: {reason}")
        self.get_logger().info("; ".join(parts))
        self._stats = {"messages": 0, "estimates": 0, "rejected": {}, "tf_failed": 0}


def transform_matrix(transform) -> np.ndarray:
    q = transform.rotation
    T = np.eye(4)
    T[:3, :3] = quaternion_to_rotation((q.x, q.y, q.z, q.w))
    T[:3, 3] = (transform.translation.x, transform.translation.y, transform.translation.z)
    return T


def keyboard_markers(layout: KeyboardLayout, frame: str, stamp) -> MarkerArray:
    """Outline of the key field, a cap per key and its label, plus the layout's markers, all in
    the keyboard frame."""
    out = MarkerArray()

    def marker(ns, mid, mtype, **kw):
        m = Marker(ns=ns, id=mid, type=mtype, action=Marker.ADD, frame_locked=True, **kw)
        m.header.frame_id, m.header.stamp = frame, stamp
        m.pose.orientation.w = 1.0
        return m

    w, h = (float(v) for v in layout.size)
    outline = marker("keyboard_outline", 0, Marker.LINE_STRIP, scale=Vector3(x=0.002),
                     color=ColorRGBA(r=0.2, g=0.8, b=1.0, a=1.0))
    # Floats only: rosidl rejects an int (0) in a float64 field with an AssertionError.
    outline.points = [Point(x=x, y=y)
                      for x, y in ((0.0, 0.0), (w, 0.0), (w, h), (0.0, h), (0.0, 0.0))]
    out.markers.append(outline)
    for i, key in enumerate(layout.keys.values()):
        cx, cy, cz = (float(v) for v in key.center)
        cap = marker("keyboard_keys", i, Marker.CUBE,
                     scale=Vector3(x=key.width, y=key.height, z=0.004),
                     color=ColorRGBA(r=0.85, g=0.85, b=0.9, a=0.6))
        cap.pose.position = Point(x=cx, y=cy, z=cz - 0.002)
        out.markers.append(cap)
        label = marker("keyboard_labels", i, Marker.TEXT_VIEW_FACING, text=key.label,
                       scale=Vector3(z=0.008), color=ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0))
        label.pose.position = Point(x=cx, y=cy, z=cz + 0.004)
        out.markers.append(label)
    for m in layout.markers.values():
        x, y, z = (float(v) for v in m.position)
        plate = marker("keyboard_markers", m.id, Marker.CUBE,
                       scale=Vector3(x=m.size, y=m.size, z=0.002),
                       color=ColorRGBA(r=0.1, g=0.9, b=0.2, a=0.8))
        plate.pose.position = Point(x=x, y=y, z=z)
        c, s = np.cos(m.yaw / 2), np.sin(m.yaw / 2)
        plate.pose.orientation = Quaternion(z=float(s), w=float(c))
        out.markers.append(plate)
    return out


def main():
    rclpy.init()
    try:
        rclpy.spin(KeyboardNode(), MultiThreadedExecutor(num_threads=2))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()
