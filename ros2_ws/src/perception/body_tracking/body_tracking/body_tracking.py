"""Recognize configured BODY_18 poses from ZED body-tracking messages."""

import rclpy
from rclpy.node import Node
from zed_msgs.msg import ObjectsStamped

from body_tracking.pose_detection import SKELETON_POSITION, classify_objects


class MinimalSubscriber(Node):
    """Classify one BODY_18 skeleton per incoming ZED message."""

    def __init__(self):
        """Configure pose classification and subscribe to ZED skeletons."""
        super().__init__('body_tracking')

        self.declare_parameter(
            "skeleton_topic", "/zed/zed_node/body_trk/skeletons")
        self.declare_parameter("angle_tolerance_degrees", 15.0)
        self.declare_parameter("level_tolerance_ratio", 0.10)
        skeleton_topic = self.get_parameter("skeleton_topic").value
        self._angle_tolerance_degrees = self.get_parameter(
            "angle_tolerance_degrees").value
        self._level_tolerance_ratio = self.get_parameter(
            "level_tolerance_ratio").value
        self._last_position = SKELETON_POSITION.UNKNOWN

        self.subscription = self.create_subscription(
            ObjectsStamped,
            skeleton_topic,
            self.listener_callback,
            1)

    def listener_callback(self, msg):
        """Log only changes to the detected BODY_18 pose."""
        position = classify_objects(
            msg.objects,
            self._angle_tolerance_degrees,
            self._level_tolerance_ratio)
        if position == self._last_position:
            return

        if position == SKELETON_POSITION.UNKNOWN:
            self.get_logger().info("No supported pose detected")
        else:
            self.get_logger().info(f"Pose detected: {position.name}")
        self._last_position = position


def main(args=None):
    """Run the body-tracking pose recognition node."""
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
