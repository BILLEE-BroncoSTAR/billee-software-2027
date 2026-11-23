"""Recognize configured BODY_18 poses from ZED body-tracking messages."""

import rclpy
from body_tracking_msgs.msg import BodyTracking
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from zed_msgs.msg import ObjectsStamped

from body_tracking.pose_detection import (
    SKELETON_POSITION,
    classify_objects,
    distance_from_objects,
)


class MinimalSubscriber(Node):
    """Classify one BODY_18 skeleton per incoming ZED message."""

    def __init__(self):
        """Configure pose classification and subscribe to ZED skeletons."""
        super().__init__('body_tracking')

        self.declare_parameter(
            "skeleton_topic", "/zed/zed_node/body_trk/skeletons")
        self.declare_parameter(
            "pose_topic", "/zed/zed_node/body_trk/pose")
        self.declare_parameter("angle_tolerance_degrees", 15.0)
        self.declare_parameter("level_tolerance_ratio", 0.10)
        skeleton_topic = self.get_parameter("skeleton_topic").value
        pose_topic = self.get_parameter("pose_topic").value
        self._angle_tolerance_degrees = self.get_parameter(
            "angle_tolerance_degrees").value
        self._level_tolerance_ratio = self.get_parameter(
            "level_tolerance_ratio").value
        self._last_position = SKELETON_POSITION.UNKNOWN

        self.subscription = self.create_subscription(
            ObjectsStamped,
            skeleton_topic,
            self.listener_callback,
            qos_profile_sensor_data)
        self._pose_publisher = self.create_publisher(
            BodyTracking, pose_topic, qos_profile_sensor_data)

    def listener_callback(self, msg):
        """Publish the current pose and depth-derived range for this frame."""
        position = classify_objects(
            msg.objects,
            self._angle_tolerance_degrees,
            self._level_tolerance_ratio)
        output = BodyTracking()
        output.header = msg.header
        output.skeleton_position = position.value
        output.distance_m = distance_from_objects(msg.objects)
        self._pose_publisher.publish(output)

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
