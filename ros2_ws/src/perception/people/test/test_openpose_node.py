import math

import numpy as np
from geometry_msgs.msg import TransformStamped
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo

from people.openpose_node import decode, lift, to_matrix


def test_decode_maps_heatmap_peak_to_image_pixels():
    heatmaps = np.zeros((18, 46, 80), np.float32)
    heatmaps[3, 10, 20] = 0.9
    keypoints = decode(heatmaps, 1280, 720)
    assert np.allclose(keypoints[3], [20 * 1280 / 80, 10 * 720 / 46, 0.9])


def test_lift_recovers_a_projected_point():
    fx = fy = 369.5
    cx, cy = 640.0, 360.0
    info = CameraInfo(width=1280, height=720, distortion_model='plumb_bob', d=[0.0] * 5,
                      k=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
                      r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                      p=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0])
    camera = PinholeCameraModel()
    camera.fromCameraInfo(info)
    point = np.array([0.5, -0.3, 4.0])
    keypoints = np.zeros((18, 3))
    keypoints[0] = [fx * point[0] / point[2] + cx, fy * point[1] / point[2] + cy, 1.0]
    keypoints[2] = [fx * 0.6 / point[2] + cx, fy * -0.3 / point[2] + cy, 1.0]
    keypoints[3] = [100.0, 100.0, 1.0]  # lands on the background
    depth = np.full((720, 1280), point[2], np.float32)
    depth[90:111, 90:111] = 9.0

    xyz = lift(keypoints, depth, camera, 0.1)

    assert np.allclose(xyz[0], point, atol=1e-3)
    assert np.isnan(xyz[1]).all()  # below the confidence threshold -> unknown
    assert np.isnan(xyz[3]).all()  # 5 m behind the body -> dropped as background


def test_to_matrix_yaw_90_turns_x_into_y():
    tf = TransformStamped()
    tf.transform.rotation.z = math.sin(math.pi / 4)
    tf.transform.rotation.w = math.cos(math.pi / 4)
    tf.transform.translation.x = 1.0
    rotation, translation = to_matrix(tf)
    assert np.allclose(rotation @ [1.0, 0.0, 0.0] + translation, [1.0, 1.0, 0.0])
