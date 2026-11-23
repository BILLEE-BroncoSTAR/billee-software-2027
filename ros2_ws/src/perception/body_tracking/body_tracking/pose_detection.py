"""Pose-classification helpers."""

from __future__ import annotations

import math
from collections.abc import Sequence
from enum import Enum

BODY_18_FORMAT = 0
ARM_JOINT_INDICES = (2, 3, 4, 5, 6, 7)
Point = tuple[float, float]


class SKELETON_POSITION(Enum):
    """The supported body poses."""

    UNKNOWN = 0
    ARMS_UP = 1
    T_POSE = 2


def keypoint_to_point(keypoint: object) -> Point | None:
    """Return a valid 2D point, or ``None`` for an unavailable keypoint."""
    try:
        x, y = keypoint.kp
    except (AttributeError, TypeError, ValueError):
        return None

    if not math.isfinite(x) or not math.isfinite(y) or x < 0.0 or y < 0.0:
        return None
    return float(x), float(y)


def arm_keypoints(keypoints: Sequence[object]) -> dict[int, Point] | None:
    """Extract the six valid BODY_18 arm keypoints needed for recognition."""
    if len(keypoints) <= max(ARM_JOINT_INDICES):
        return None

    points: dict[int, Point] = {}
    for index in ARM_JOINT_INDICES:
        point = keypoint_to_point(keypoints[index])
        if point is None:
            return None
        points[index] = point
    return points


def segment_length(first: Point, second: Point) -> float:
    """Return the Euclidean length of a 2D segment."""
    return math.dist(first, second)


def joint_angle(first: Point, vertex: Point, third: Point) -> float | None:
    """Return the angle in degrees at ``vertex``, or ``None`` if degenerate."""
    first_vector = (first[0] - vertex[0], first[1] - vertex[1])
    third_vector = (third[0] - vertex[0], third[1] - vertex[1])
    first_length = math.hypot(*first_vector)
    third_length = math.hypot(*third_vector)
    if first_length == 0.0 or third_length == 0.0:
        return None

    cosine = (
        first_vector[0] * third_vector[0]
        + first_vector[1] * third_vector[1]
    ) / (first_length * third_length)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def angle_matches(
    angle: float | None, target_degrees: float, tolerance_degrees: float
) -> bool:
    """Return whether an angle falls within an inclusive target tolerance."""
    return (
        angle is not None
        and tolerance_degrees >= 0.0
        and abs(angle - target_degrees) <= tolerance_degrees
    )


def is_level(
    first: Point,
    second: Point,
    reference_length: float,
    tolerance_ratio: float,
) -> bool:
    """Return whether two image points have nearly equal vertical positions."""
    return (
        reference_length > 0.0
        and tolerance_ratio >= 0.0
        and abs(first[1] - second[1]) <= reference_length * tolerance_ratio
    )


def classify_keypoints(
    keypoints: Sequence[object],
    angle_tolerance_degrees: float = 15.0,
    level_tolerance_ratio: float = 0.10,
) -> SKELETON_POSITION:
    """Classify BODY_18 arm keypoints as T-pose, goalpost, or unknown."""
    points = arm_keypoints(keypoints)
    if points is None:
        return SKELETON_POSITION.UNKNOWN

    right_shoulder, right_elbow, right_wrist = (
        points[2], points[3], points[4])
    left_shoulder, left_elbow, left_wrist = (
        points[5], points[6], points[7])
    right_upper_arm_length = segment_length(right_shoulder, right_elbow)
    left_upper_arm_length = segment_length(left_shoulder, left_elbow)
    right_angle = joint_angle(right_shoulder, right_elbow, right_wrist)
    left_angle = joint_angle(left_shoulder, left_elbow, left_wrist)

    right_arm_level = is_level(
        right_shoulder, right_elbow, right_upper_arm_length,
        level_tolerance_ratio)
    left_arm_level = is_level(
        left_shoulder, left_elbow, left_upper_arm_length,
        level_tolerance_ratio)

    if (
        angle_matches(right_angle, 180.0, angle_tolerance_degrees)
        and angle_matches(left_angle, 180.0, angle_tolerance_degrees)
        and right_arm_level
        and left_arm_level
        and is_level(
            right_shoulder, right_wrist, right_upper_arm_length,
            level_tolerance_ratio)
        and is_level(
            left_shoulder, left_wrist, left_upper_arm_length,
            level_tolerance_ratio)
    ):
        return SKELETON_POSITION.T_POSE

    if (
        angle_matches(right_angle, 90.0, angle_tolerance_degrees)
        and angle_matches(left_angle, 90.0, angle_tolerance_degrees)
        and right_arm_level
        and left_arm_level
        and right_wrist[1] < right_elbow[1]
        and left_wrist[1] < left_elbow[1]
    ):
        return SKELETON_POSITION.ARMS_UP

    return SKELETON_POSITION.UNKNOWN


def classify_body(
    body: object,
    angle_tolerance_degrees: float = 15.0,
    level_tolerance_ratio: float = 0.10,
) -> SKELETON_POSITION:
    """Classify one available BODY_18 body-tracking object."""
    if (
        not getattr(body, "skeleton_available", False)
        or getattr(body, "body_format", None) != BODY_18_FORMAT
    ):
        return SKELETON_POSITION.UNKNOWN

    skeleton_2d = getattr(body, "skeleton_2d", None)
    keypoints = getattr(skeleton_2d, "keypoints", None)
    if keypoints is None:
        return SKELETON_POSITION.UNKNOWN
    return classify_keypoints(
        keypoints, angle_tolerance_degrees, level_tolerance_ratio)


def classify_objects(
    objects: Sequence[object],
    angle_tolerance_degrees: float = 15.0,
    level_tolerance_ratio: float = 0.10,
) -> SKELETON_POSITION:
    """Classify a frame only when it contains exactly one tracked person."""
    if len(objects) != 1:
        return SKELETON_POSITION.UNKNOWN
    return classify_body(
        objects[0], angle_tolerance_degrees, level_tolerance_ratio)


def distance_from_objects(objects: Sequence[object]) -> float:
    """
    Return the depth-derived range to one object's centroid in metres.

    ZED's ``Object.position`` is a three-dimensional centroid derived from
    depth data. A frame without exactly one finite centroid has no usable
    range, represented by ``NaN``.
    """
    if len(objects) != 1:
        return math.nan

    try:
        x, y, z = objects[0].position
    except (AttributeError, TypeError, ValueError):
        return math.nan

    if not all(math.isfinite(coordinate) for coordinate in (x, y, z)):
        return math.nan
    return math.sqrt(x * x + y * y + z * z)
