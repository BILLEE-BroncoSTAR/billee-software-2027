import numpy as np
import pytest
from people.gestures import (
    ARMS_U,
    HIPS,
    LEFT_ARM_UP,
    NONE,
    RIGHT_ARM_UP,
    T_POSE,
    HoldFilter,
    classify,
)

SHOULDER_WIDTH = 100.0  # px; the torso is 2 shoulder widths so both scale estimates agree
ARMS_DOWN = ((0.1, -0.8), (0.1, -1.5))


def person(right=ARMS_DOWN, left=ARMS_DOWN, facing_camera=True, model='BODY_25'):
    """OpenPose (u, v, conf) for `model`. Each arm is ((elbow out, up), (wrist out, up)) from its
    shoulder, in shoulder widths. Facing the camera, the person's right side is on the image left."""
    n = {'BODY_25': 25, 'COCO': 18}[model]
    u, v, conf = np.zeros(n), np.zeros(n), np.ones(n)
    r_hip, l_hip = HIPS[model]
    neck = np.array([640.0, 300.0])
    u[0], v[0] = neck + [0, -60]
    u[1], v[1] = neck
    right_side = -1 if facing_camera else 1
    for side, (shoulder, elbow, wrist), (hip,), arm in (
            (right_side, (2, 3, 4), (r_hip,), right), (-right_side, (5, 6, 7), (l_hip,), left)):
        su, sv = neck + [side * SHOULDER_WIDTH / 2, 0]
        u[shoulder], v[shoulder] = su, sv
        u[hip], v[hip] = neck + [side * 40, 2 * SHOULDER_WIDTH]
        for joint, (out, up) in zip((elbow, wrist), arm):
            u[joint] = su + side * out * SHOULDER_WIDTH
            v[joint] = sv - up * SHOULDER_WIDTH
    return u, v, conf


GOALPOST = ((0.8, 0.0), (0.8, 0.9))
STRAIGHT_UP = ((0.1, 0.8), (0.15, 1.6))
WIDE_Y = ((0.6, 0.6), (1.2, 1.2))
OUT_FLAT = ((0.8, 0.0), (1.6, 0.05))


@pytest.mark.parametrize('right, left, facing, expected', [
    (ARMS_DOWN, ARMS_DOWN, True, NONE),
    (GOALPOST, GOALPOST, True, ARMS_U),
    (STRAIGHT_UP, STRAIGHT_UP, True, ARMS_U),
    (GOALPOST, GOALPOST, False, ARMS_U),
    (WIDE_Y, WIDE_Y, True, NONE),
    (OUT_FLAT, OUT_FLAT, True, T_POSE),
    (STRAIGHT_UP, ARMS_DOWN, True, RIGHT_ARM_UP),
    (ARMS_DOWN, STRAIGHT_UP, True, LEFT_ARM_UP),
    (STRAIGHT_UP, ARMS_DOWN, False, RIGHT_ARM_UP),  # sides swap in the image; still their right
    (GOALPOST, OUT_FLAT, True, NONE),
])
@pytest.mark.parametrize('model', ['BODY_25', 'COCO'])
def test_classify(right, left, facing, expected, model):
    gesture, confidence = classify(*person(right, left, facing, model), model=model)
    assert gesture == expected
    assert confidence == pytest.approx(1.0)


def test_body_25_uses_its_own_hip_indices():
    # BODY_25 index 8 is the mid hip, not the right hip as in COCO: with the hips 4 shoulder widths
    # below the neck, half the torso (2) must win over the shoulder width (1) as the scale.
    u, v, conf = person(OUT_FLAT, OUT_FLAT, model='BODY_25')
    assert classify(u, v, conf, model='BODY_25')[0] == T_POSE
    v[[9, 12]] += 2 * SHOULDER_WIDTH  # move only BODY_25's right/left hip
    assert classify(u, v, conf, model='BODY_25')[0] == NONE  # arms now too short for a T-pose


def test_right_arm_up_is_on_the_image_left_when_facing_the_camera():
    u, _, _ = person(STRAIGHT_UP, ARMS_DOWN)
    assert u[4] < u[1]


def test_unseen_arm_joint_means_no_gesture():
    u, v, conf = person(GOALPOST, GOALPOST)
    conf[7] = 0.05  # left wrist
    assert classify(u, v, conf) == (NONE, 0.0)


def test_hold_filter_needs_consecutive_frames_both_ways():
    hold = HoldFilter(2)
    assert [hold.update(g) for g in (ARMS_U, NONE, ARMS_U, ARMS_U, ARMS_U, NONE, NONE)] == \
        [NONE, NONE, NONE, ARMS_U, ARMS_U, ARMS_U, NONE]
