"""Turns /people/skeletons into /people/gesture: which arm gesture the person closest to the rover
holds, and the command it maps to (config/gestures.yaml) for whatever acts on it, e.g. navigation.

Also draws the gesture as a text label above that person (/people/gesture_marker) for Foxglove.
"""

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from people_msgs.msg import Gesture, Skeletons
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from people.gestures import (
    GESTURES,
    L_SHOULDER,
    NECK,
    NONE,
    R_SHOULDER,
    HoldFilter,
    classify,
)

LABEL_HEIGHT = 0.6  # m above the neck


def xyz(point):
    return np.array([point.x, point.y, point.z], dtype=float)


def person_position(skeleton):
    """Neck if known, else the centroid; NaN if neither is."""
    if len(skeleton.joints) > NECK and np.isfinite(xyz(skeleton.joints[NECK])).all():
        return xyz(skeleton.joints[NECK])
    return xyz(skeleton.centroid)


def closest_person(skeletons):
    """The person nearest the frame origin (the rover); without 3D, the widest-shouldered one."""
    def distance(skeleton):
        d = np.linalg.norm(person_position(skeleton))
        if np.isfinite(d):
            return d
        u, v = skeleton.u, skeleton.v
        if len(u) > L_SHOULDER:
            return 1e6 - np.hypot(u[R_SHOULDER] - u[L_SHOULDER], v[R_SHOULDER] - v[L_SHOULDER])
        return np.inf
    return min(skeletons.people, key=distance, default=None)


class GestureNode(Node):

    def __init__(self):
        super().__init__('gesture_node')
        self.conf_threshold = self.declare_parameter('conf_threshold', 0.2).value
        self.hold = HoldFilter(self.declare_parameter('hold_frames', 2).value)
        self.commands = {g: self.declare_parameter(f'commands.{g}', '').value for g in GESTURES}

        self.gesture_pub = self.create_publisher(Gesture, '/people/gesture', 10)
        self.marker_pub = self.create_publisher(Marker, '/people/gesture_marker', 1)
        self.create_subscription(Skeletons, '/people/skeletons', self.on_skeletons, 10)
        mapping = ', '.join(f'{g} -> {c!r}' for g, c in self.commands.items())
        self.get_logger().info(f'Waiting for /people/skeletons; {mapping}')

    def on_skeletons(self, skeletons):
        person = closest_person(skeletons)
        seen, confidence = NONE, 0.0
        position = np.full(3, np.nan)
        if person is not None:
            seen, confidence = classify(person.u, person.v, person.confidence,
                                        self.conf_threshold, skeletons.model)
            position = person_position(person)
        previous = self.hold.current
        gesture = self.hold.update(seen)
        command = self.commands.get(gesture, '')
        if gesture != previous:
            self.get_logger().info(f'Gesture {previous} -> {gesture}'
                                   + (f' (command {command!r})' if command else ''))

        x, y, z = position
        msg = Gesture(header=skeletons.header, gesture=gesture, command=command,
                      confidence=confidence if seen == gesture else 0.0,
                      position=Point(x=x, y=y, z=z))
        self.gesture_pub.publish(msg)
        self.publish_label(msg)

    def publish_label(self, msg):
        marker = Marker(header=msg.header, ns='gesture', id=0)
        p = msg.position
        if msg.gesture == NONE or not np.isfinite([p.x, p.y, p.z]).all():
            marker.action = Marker.DELETE
        else:
            marker.type = Marker.TEXT_VIEW_FACING
            marker.text = msg.gesture + (f' -> {msg.command}' if msg.command else '')
            marker.pose.position = Point(x=p.x, y=p.y, z=p.z + LABEL_HEIGHT)
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.25
            marker.color = ColorRGBA(r=1.0, g=0.85, b=0.1, a=1.0)
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    try:
        rclpy.spin(GestureNode())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
