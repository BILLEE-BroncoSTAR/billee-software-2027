"""Rule-based arm gestures from OpenPose 2D keypoints, plus a hold filter against flicker.

Measured in the image, relative to the neck and in shoulder widths, so it works at any distance
and doesn't need depth (a raised hand often has none). Assumes a roughly level camera and an
upright person facing it or facing away (each arm's "outward" side is taken from its shoulder).
"""

import numpy as np

NONE = 'none'
ARMS_U = 'arms_u'
T_POSE = 't_pose'
RIGHT_ARM_UP = 'right_arm_up'
LEFT_ARM_UP = 'left_arm_up'
GESTURES = (ARMS_U, T_POSE, RIGHT_ARM_UP, LEFT_ARM_UP)

# Neck and arms have the same indices in OpenPose's BODY_25, COCO and MPI; the hips don't.
NECK = 1
R_SHOULDER, R_ELBOW, R_WRIST = 2, 3, 4
L_SHOULDER, L_ELBOW, L_WRIST = 5, 6, 7
ARM_JOINTS = [NECK, R_SHOULDER, R_ELBOW, R_WRIST, L_SHOULDER, L_ELBOW, L_WRIST]
HIPS = {'BODY_25': (9, 12), 'COCO': (8, 11), 'MPI': (8, 11)}  # (right, left)


def arm_pose(u, v, scale, shoulder, elbow, wrist):
    """Elbow/wrist height above the shoulder and distance out from the neck, in shoulder widths."""
    outward = np.sign(u[shoulder] - u[NECK]) or 1.0
    return {
        'elbow_up': (v[shoulder] - v[elbow]) / scale,
        'wrist_up': (v[shoulder] - v[wrist]) / scale,
        'elbow_out': outward * (u[elbow] - u[NECK]) / scale,
        'wrist_out': outward * (u[wrist] - u[NECK]) / scale,
    }


def is_u_arm(a):
    # Hand well above the shoulder, forearm roughly vertical (wrist over the elbow), elbow not
    # hanging: covers goalpost arms and arms straight up, but not a wide "Y".
    return (a['wrist_up'] > 0.8 and a['elbow_up'] > -0.3 and a['wrist_out'] > 0.2
            and abs(a['wrist_out'] - a['elbow_out']) < 0.45)


def is_t_arm(a):
    return abs(a['wrist_up']) < 0.4 and abs(a['elbow_up']) < 0.4 and a['wrist_out'] > 1.4


def is_raised(a):
    return a['wrist_up'] > 1.0


def is_down(a):
    return a['wrist_up'] < 0.0


def classify(u, v, confidence, conf_threshold=0.2, model='BODY_25'):
    """OpenPose keypoints (pixels, image v down) -> (gesture, lowest confidence of the arm joints)."""
    u, v, confidence = (np.asarray(x, dtype=float) for x in (u, v, confidence))
    if len(confidence) <= max(ARM_JOINTS) or (confidence[ARM_JOINTS] < conf_threshold).any():
        return NONE, 0.0
    scale = np.hypot(u[R_SHOULDER] - u[L_SHOULDER], v[R_SHOULDER] - v[L_SHOULDER])
    r_hip, l_hip = HIPS.get(model, HIPS['BODY_25'])
    if len(confidence) > l_hip and min(confidence[r_hip], confidence[l_hip]) >= conf_threshold:
        # Shoulder width shrinks when the person turns; half the torso length doesn't.
        hips = (np.array([u[r_hip], v[r_hip]]) + [u[l_hip], v[l_hip]]) / 2
        scale = max(scale, 0.5 * np.hypot(*(hips - [u[NECK], v[NECK]])))
    if scale < 1.0:
        return NONE, 0.0
    right = arm_pose(u, v, scale, R_SHOULDER, R_ELBOW, R_WRIST)
    left = arm_pose(u, v, scale, L_SHOULDER, L_ELBOW, L_WRIST)
    conf = float(confidence[ARM_JOINTS].min())

    if is_u_arm(right) and is_u_arm(left):
        return ARMS_U, conf
    if is_t_arm(right) and is_t_arm(left):
        return T_POSE, conf
    if is_raised(right) and is_down(left):
        return RIGHT_ARM_UP, conf
    if is_raised(left) and is_down(right):
        return LEFT_ARM_UP, conf
    return NONE, conf


class HoldFilter:
    """Switches to a new gesture (including back to none) only after hold_frames in a row."""

    def __init__(self, hold_frames):
        self.hold_frames = max(1, hold_frames)
        self.current = NONE
        self.candidate = NONE
        self.count = 0

    def update(self, gesture):
        if gesture == self.candidate:
            self.count += 1
        else:
            self.candidate, self.count = gesture, 1
        if self.count >= self.hold_frames:
            self.current = gesture
        return self.current
