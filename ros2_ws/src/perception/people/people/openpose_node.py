"""OpenPose body keypoints from the camera, lifted to 3D and placed in odom.

Mirrors stereolabs/zed-openpose: 2D OpenPose skeleton on the colour image, depth under each
keypoint -> 3D point in the camera optical frame, then TF into odom (where the rover believes
its camera is). Publishes /people/debug_image and /people/markers for Foxglove.
"""

import os

import cv2
import message_filters
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from image_geometry import PinholeCameraModel
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

# OpenPose COCO-18 keypoint order (same as ZED BODY_18 and trt_pose):
# 0 nose, 1 neck, 2-4 right shoulder/elbow/wrist, 5-7 left shoulder/elbow/wrist,
# 8-10 right hip/knee/ankle, 11-13 left hip/knee/ankle, 14/15 right/left eye, 16/17 right/left ear
NUM_KEYPOINTS = 18
LIMBS = [(1, 2), (1, 5), (2, 3), (3, 4), (5, 6), (6, 7), (1, 8), (8, 9), (9, 10),
         (1, 11), (11, 12), (12, 13), (1, 0), (0, 14), (14, 16), (0, 15), (15, 17)]
LIMB_COLORS = [tuple(int(c) for c in cv2.cvtColor(
    np.uint8([[[i * 180 // len(LIMBS), 255, 255]]]), cv2.COLOR_HSV2BGR)[0, 0])
    for i in range(len(LIMBS))]

PROTOTXT = 'pose_deploy_linevec.prototxt'
WEIGHTS = 'pose_iter_440000.caffemodel'


def decode(heatmaps, width, height):
    """(18, h, w) heatmaps -> (18, 3) [u, v, conf] in image pixels."""
    # ponytail: single person via the strongest peak per joint; add PAF grouping for multi-person.
    n, h, w = heatmaps.shape
    flat = heatmaps.reshape(n, -1)
    idx = flat.argmax(axis=1)
    v, u = np.divmod(idx, w)
    return np.stack([u * width / w, v * height / h, flat[np.arange(n), idx]], axis=1)


def lift(keypoints, depth, model, conf_threshold, patch=2, max_spread=0.75):
    """Keypoints (18, 3) + depth image (m) -> (18, 3) xyz in the optical frame, NaN if unknown.

    Joints more than max_spread (m) from the body's median depth are dropped: on thin limbs the
    depth patch can miss the person and hit the background behind them.
    """
    xyz = np.full((len(keypoints), 3), np.nan)
    for i, (u, v, conf) in enumerate(keypoints):
        if conf < conf_threshold:
            continue
        x, y = int(round(u)), int(round(v))
        window = depth[max(y - patch, 0):y + patch + 1, max(x - patch, 0):x + patch + 1]
        window = window[np.isfinite(window) & (window > 0)]
        if window.size == 0:
            continue
        ray = np.array(model.projectPixelTo3dRay(model.rectifyPoint((u, v))))
        xyz[i] = ray / ray[2] * float(np.median(window))
    z = xyz[:, 2]
    if np.isfinite(z).any():
        xyz[np.abs(z - np.nanmedian(z)) > max_spread] = np.nan
    return xyz


def to_matrix(transform):
    """geometry_msgs/TransformStamped -> (3x3 rotation, translation)."""
    q, t = transform.transform.rotation, transform.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return rotation, np.array([t.x, t.y, t.z])


class OpenPoseNode(Node):

    def __init__(self):
        super().__init__('openpose_node')
        p = self.declare_parameter
        image_topic = p('image_topic', '/depth_cam/image').value
        depth_topic = p('depth_topic', '/depth_cam/depth_image').value
        info_topic = p('info_topic', '/depth_cam/camera_info').value
        # The sim camera labels its images camera_link, but back-projected points use the
        # optical convention (z forward), so they are stamped with the optical frame.
        self.optical_frame = p('optical_frame', 'camera_link_optical').value
        self.target_frame = p('target_frame', 'odom').value
        model_dir = p('model_dir', os.path.join(
            os.environ.get('PIXI_PROJECT_ROOT', '.'), 'models', 'openpose')).value
        self.net_height = p('net_height', 368).value
        self.conf_threshold = p('conf_threshold', 0.1).value

        prototxt, weights = (os.path.join(model_dir, f) for f in (PROTOTXT, WEIGHTS))
        if not (os.path.isfile(prototxt) and os.path.isfile(weights)):
            raise SystemExit(f'OpenPose model not found in {model_dir}; run '
                             'src/perception/people/scripts/get_openpose_models.sh')
        self.net = cv2.dnn.readNetFromCaffe(prototxt, weights)

        self.bridge = CvBridge()
        self.camera = PinholeCameraModel()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.image_pub = self.create_publisher(Image, '/people/debug_image', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/people/markers', 1)

        # Depth 1: inference is slower than the camera, so only the newest frame is kept.
        subs = [message_filters.Subscriber(self, Image, image_topic, qos_profile=1),
                message_filters.Subscriber(self, Image, depth_topic, qos_profile=1),
                message_filters.Subscriber(self, CameraInfo, info_topic, qos_profile=1)]
        self.sync = message_filters.ApproximateTimeSynchronizer(subs, queue_size=2, slop=0.05)
        self.sync.registerCallback(self.on_frame)
        self.get_logger().info(f'OpenPose loaded from {model_dir}; waiting for {image_topic}')

    def detect(self, bgr):
        h, w = bgr.shape[:2]
        net_w = int(self.net_height * w / h)
        self.net.setInput(cv2.dnn.blobFromImage(
            bgr, 1.0 / 255, (net_w, self.net_height), (0, 0, 0), swapRB=False, crop=False))
        # Heatmaps come out at 1/8 of the input; upsample (as OpenPose does) for sub-cell peaks.
        heatmaps = np.stack([cv2.resize(m, (net_w, self.net_height), interpolation=cv2.INTER_CUBIC)
                             for m in self.net.forward()[0, :NUM_KEYPOINTS]])
        return decode(heatmaps, w, h)

    def on_frame(self, image_msg, depth_msg, info_msg):
        bgr = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)
        if depth_msg.encoding == '16UC1':  # millimetres (real cameras); the sim sends 32FC1 metres
            depth /= 1000.0
        self.camera.fromCameraInfo(info_msg)

        keypoints = self.detect(bgr)
        self.publish_debug_image(bgr, keypoints, image_msg.header)

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, self.optical_frame, Time.from_msg(image_msg.header.stamp),
                timeout=Duration(seconds=0.1))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'No TF {self.target_frame} <- {self.optical_frame}: {e}',
                                   throttle_duration_sec=5.0)
            return
        rotation, translation = to_matrix(tf)
        points = lift(keypoints, depth, self.camera, self.conf_threshold) @ rotation.T + translation
        self.publish_markers(points, image_msg.header.stamp)

    def publish_debug_image(self, bgr, keypoints, header):
        for (a, b), color in zip(LIMBS, LIMB_COLORS):
            if min(keypoints[a, 2], keypoints[b, 2]) >= self.conf_threshold:
                cv2.line(bgr, tuple(map(int, keypoints[a, :2])), tuple(map(int, keypoints[b, :2])),
                         color, 3, cv2.LINE_AA)
        for u, v, conf in keypoints:
            if conf >= self.conf_threshold:
                cv2.circle(bgr, (int(u), int(v)), 4, (0, 0, 255), -1, cv2.LINE_AA)
        msg = self.bridge.cv2_to_imgmsg(bgr, 'bgr8')
        msg.header = header
        self.image_pub.publish(msg)

    def publish_markers(self, points, stamp):
        red = ColorRGBA(r=1.0, g=0.15, b=0.1, a=1.0)
        limbs = Marker(type=Marker.LINE_LIST, ns='skeleton', id=0, color=red)
        limbs.scale.x = 0.03
        limbs.points = [Point(x=x, y=y, z=z) for a, b in LIMBS
                        if np.isfinite(points[[a, b]]).all() for x, y, z in points[[a, b]]]
        joints = Marker(type=Marker.SPHERE_LIST, ns='joints', id=1, color=red)
        joints.scale.x = joints.scale.y = joints.scale.z = 0.06
        joints.points = [Point(x=x, y=y, z=z) for x, y, z in points if np.isfinite(x)]
        # No lifetime: on a CPU a new skeleton can take seconds, so each one stays until the next
        # replaces it (same ns/id). An empty points list clears it when nobody is detected.
        for m in (limbs, joints):
            m.header.frame_id = self.target_frame
            m.header.stamp = stamp
            m.pose.orientation.w = 1.0
        self.marker_pub.publish(MarkerArray(markers=[limbs, joints]))


def main():
    rclpy.init()
    try:
        rclpy.spin(OpenPoseNode())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
