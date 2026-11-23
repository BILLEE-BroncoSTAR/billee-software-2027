"""Render ZED body skeletons over synchronized camera images."""

from __future__ import annotations

import math

import cv2
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from zed_msgs.msg import ObjectsStamped


# OpenPose / ZED BODY_18 joint connections.
BODY_18_EDGES = (
    (0, 1), (1, 2), (2, 3), (3, 4), (1, 5), (5, 6), (6, 7),
    (1, 8), (8, 9), (9, 10), (1, 11), (11, 12), (12, 13),
    (0, 14), (14, 16), (0, 15), (15, 17),
)
BODY_34_EDGES = (
    (0, 1), (1, 2), (2, 4), (4, 5), (5, 6), (6, 7), (7, 8),
    (8, 9), (7, 10), (2, 11), (11, 12), (12, 13), (13, 14),
    (14, 15), (15, 16), (14, 17), (0, 18), (18, 19), (19, 20),
    (20, 21), (0, 22), (22, 23), (23, 24), (24, 25), (2, 3),
    (3, 26), (26, 27), (27, 28), (28, 29), (27, 30), (30, 31),
    (20, 32), (24, 33), (32, 21), (33, 25),
)
# zed_msgs/Object.body_format values. Skeleton2D.keypoints is always a
# fixed-size, 70-element ROS array; only the first N entries are meaningful.
BODY_FORMATS = {
    0: (18, BODY_18_EDGES),  # POSE_18
    1: (34, BODY_34_EDGES),  # POSE_34
}
SKELETON_COLOURS = (
    (255, 76, 76), (255, 153, 51), (255, 255, 51), (153, 255, 51),
    (76, 255, 76), (76, 255, 153), (76, 255, 255), (76, 153, 255),
)


class ZedBodySkeletonOverlay(Node):
    """Publish each ZED image with skeletons from the same camera frame."""

    def __init__(self) -> None:
        """Subscribe to ZED images and body tracking, then configure output."""
        super().__init__("zed_body_skeleton_overlay")
        self.declare_parameter(
            "image_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter(
            "skeleton_topic", "/zed/zed_node/body_trk/skeletons")
        self.declare_parameter(
            "overlay_topic", "/zed/zed_node/body_trk/overlay")
        self.declare_parameter("sync_queue_size", 30)
        self.declare_parameter("sync_slop_s", 0.05)

        self._bridge = CvBridge()
        image_topic = self.get_parameter("image_topic").value
        skeleton_topic = self.get_parameter("skeleton_topic").value
        overlay_topic = self.get_parameter("overlay_topic").value
        queue_size = self.get_parameter("sync_queue_size").value
        slop = self.get_parameter("sync_slop_s").value

        self._image_subscriber = message_filters.Subscriber(
            self, Image, image_topic, qos_profile=qos_profile_sensor_data)
        self._skeleton_subscriber = message_filters.Subscriber(
            self, ObjectsStamped, skeleton_topic,
            qos_profile=qos_profile_sensor_data)
        self._synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self._image_subscriber, self._skeleton_subscriber], queue_size,
            slop)
        self._synchronizer.registerCallback(self._on_synchronized_messages)
        self._publisher = self.create_publisher(
            Image, overlay_topic, qos_profile_sensor_data)
        self._unsupported_body_formats: set[int] = set()
        self.get_logger().info(
            f"Synchronizing {skeleton_topic} with {image_topic} "
            f"(slop {slop:.3f}s); publishing {overlay_topic}")

    def _on_synchronized_messages(
        self, image_message: Image, bodies_message: ObjectsStamped
    ) -> None:
        """Draw poses only on their matching source image."""
        try:
            image = self._bridge.imgmsg_to_cv2(
                image_message, desired_encoding="bgr8")
        except CvBridgeError as error:
            self.get_logger().warning(f"Could not convert ZED image: {error}")
            return

        overlay = image.copy()
        for body_index, body in enumerate(bodies_message.objects):
            if body.skeleton_available:
                self._draw_body(
                    overlay, body.skeleton_2d.keypoints, body.body_format,
                    body_index)

        output = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
        output.header = image_message.header
        self._publisher.publish(output)

    @staticmethod
    def _point(
        keypoint: object, image_width: int, image_height: int
    ) -> tuple[int, int] | None:
        """Return a valid image point, excluding unavailable keypoints."""
        x, y = keypoint.kp
        if (
            not math.isfinite(x)
            or not math.isfinite(y)
            or x < 0.0
            or y < 0.0
            or x >= image_width
            or y >= image_height
        ):
            return None
        return round(x), round(y)

    def _draw_body(
        self, image, keypoints, body_format: int, body_index: int
    ) -> None:
        """Draw only the keypoints and limbs valid for ZED's pose format."""
        pose_description = BODY_FORMATS.get(body_format)
        if pose_description is None:
            if body_format not in self._unsupported_body_formats:
                self.get_logger().warning(
                    f"Skipping unsupported ZED body format {body_format}; "
                    "this overlay supports POSE_18 and POSE_34.")
                self._unsupported_body_formats.add(body_format)
            return

        keypoint_count, edges = pose_description
        image_height, image_width = image.shape[:2]
        points = [
            self._point(keypoint, image_width, image_height)
            for keypoint in keypoints[:keypoint_count]
        ]
        body_colour_offset = body_index % len(SKELETON_COLOURS)
        for edge_index, (first, second) in enumerate(edges):
            if points[first] is not None and points[second] is not None:
                colour = SKELETON_COLOURS[
                    (edge_index + body_colour_offset) % len(SKELETON_COLOURS)]
                cv2.line(
                    image, points[first], points[second], colour, 3,
                    cv2.LINE_AA)
        for point in points:
            if point is not None:
                cv2.circle(image, point, 4, (255, 255, 255), -1, cv2.LINE_AA)


def main() -> None:
    """Run the overlay node until it is interrupted."""
    import rclpy

    rclpy.init()
    node = ZedBodySkeletonOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
