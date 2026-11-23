"""Tests for BODY_18 pose recognition."""

from math import nan
from types import SimpleNamespace

import pytest
from body_tracking.pose_detection import (
    SKELETON_POSITION,
    angle_matches,
    classify_body,
    classify_keypoints,
    classify_objects,
    distance_from_objects,
)


class Keypoint:
    """A minimal stand-in for ZED's 2D keypoint message type."""

    def __init__(self, x: float, y: float) -> None:
        """Store the two-dimensional keypoint coordinate."""
        self.kp = (x, y)


def make_keypoints(points: dict[int, tuple[float, float]]) -> list[Keypoint]:
    """Build a complete BODY_18 keypoint array with selected replacements."""
    keypoints = [Keypoint(100.0, 100.0) for _ in range(18)]
    for index, (x, y) in points.items():
        keypoints[index] = Keypoint(x, y)
    return keypoints


def make_body(
    keypoints: list[Keypoint],
    available: bool = True,
    body_format: int = 0,
    position: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> SimpleNamespace:
    """Build the portion of a ZED body object used by classification."""
    return SimpleNamespace(
        skeleton_available=available,
        body_format=body_format,
        position=position,
        skeleton_2d=SimpleNamespace(keypoints=keypoints),
    )


T_POSE_POINTS = {
    2: (20.0, 50.0), 3: (40.0, 50.0), 4: (60.0, 50.0),
    5: (80.0, 50.0), 6: (60.0, 50.0), 7: (40.0, 50.0),
}
ARMS_UP_POINTS = {
    2: (20.0, 50.0), 3: (40.0, 50.0), 4: (40.0, 30.0),
    5: (80.0, 50.0), 6: (60.0, 50.0), 7: (60.0, 30.0),
}


def test_classifies_a_level_t_pose() -> None:
    """Straight, level arms form a T-pose."""
    position = classify_keypoints(make_keypoints(T_POSE_POINTS))
    assert position == SKELETON_POSITION.T_POSE


def test_classifies_a_symmetric_goalpost_pose() -> None:
    """Level upper arms and raised forearms form ARMS_UP."""
    position = classify_keypoints(make_keypoints(ARMS_UP_POINTS))
    assert position == SKELETON_POSITION.ARMS_UP


@pytest.mark.parametrize(
    "points",
    [
        {**T_POSE_POINTS, 4: (60.0, 55.0)},
        {**T_POSE_POINTS, 4: (50.0, 65.0)},
        {**ARMS_UP_POINTS, 7: (40.0, 50.0)},
    ],
)
def test_rejects_uneven_or_non_goalpost_arms(
    points: dict[int, tuple[float, float]]
) -> None:
    """Bent, uneven, and unraised arms are not recognized poses."""
    position = classify_keypoints(make_keypoints(points))
    assert position == SKELETON_POSITION.UNKNOWN


@pytest.mark.parametrize(
    ("angle", "expected"),
    [(165.0, True), (195.0, True), (164.99, False), (195.01, False)],
)
def test_angle_tolerance_boundaries(angle: float, expected: bool) -> None:
    """The selected 15-degree angle tolerance is inclusive at its bounds."""
    assert angle_matches(angle, 180.0, 15.0) is expected


@pytest.mark.parametrize("invalid_point", [(nan, 50.0), (-1.0, 50.0)])
def test_rejects_invalid_required_keypoints(
    invalid_point: tuple[float, float]
) -> None:
    """Unavailable and non-finite keypoints prevent a classification."""
    points = {**T_POSE_POINTS, 4: invalid_point}
    position = classify_keypoints(make_keypoints(points))
    assert position == SKELETON_POSITION.UNKNOWN


def test_rejects_unavailable_or_unsupported_body() -> None:
    """Only available BODY_18 skeletons are eligible for classification."""
    keypoints = make_keypoints(T_POSE_POINTS)
    assert classify_body(make_body(keypoints, available=False)) == (
        SKELETON_POSITION.UNKNOWN)
    assert classify_body(make_body(keypoints, body_format=1)) == (
        SKELETON_POSITION.UNKNOWN)


@pytest.mark.parametrize(
    "bodies", [[], [make_body(make_keypoints(T_POSE_POINTS))] * 2])
def test_requires_exactly_one_tracked_person(
    bodies: list[SimpleNamespace]
) -> None:
    """A frame must contain exactly one object before it is classified."""
    assert classify_objects(bodies) == SKELETON_POSITION.UNKNOWN


def test_returns_depth_derived_centroid_distance() -> None:
    """The published range is the Euclidean distance to ZED's centroid."""
    body = make_body(make_keypoints(T_POSE_POINTS), position=(3.0, 4.0, 12.0))
    assert distance_from_objects([body]) == 13.0


@pytest.mark.parametrize(
    "bodies",
    [[], [make_body(make_keypoints(T_POSE_POINTS))] * 2],
)
def test_marks_distance_unavailable_without_exactly_one_body(
    bodies: list[SimpleNamespace]
) -> None:
    """No unique object centroid yields an unavailable distance."""
    assert distance_from_objects(bodies) != distance_from_objects(bodies)


def test_marks_distance_unavailable_for_non_finite_centroid() -> None:
    """Invalid depth coordinates do not produce a range."""
    body = make_body(make_keypoints(T_POSE_POINTS), position=(nan, 0.0, 0.0))
    assert distance_from_objects([body]) != distance_from_objects([body])
