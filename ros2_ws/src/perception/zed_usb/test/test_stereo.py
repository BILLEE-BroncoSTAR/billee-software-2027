"""zed_usb's stereo maths, with the team's real ZED 2i calibration and synthetic frames (no camera)."""
import os
import struct

import cv2
import numpy as np
import pytest

from zed_usb import stereo

HERE = os.path.dirname(os.path.abspath(__file__))
TEAM_CONF = os.path.normpath(os.path.join(HERE, '..', '..', '..', '..', '..', 'configs', 'zed', 'SN36243153.conf'))
needs_conf = pytest.mark.skipif(not os.path.isfile(TEAM_CONF), reason='configs/zed/SN36243153.conf not found')


def fake_sysfs(tmp_path, video, usb):
    for name, label in video.items():
        d = tmp_path / 'class' / 'video4linux' / name
        d.mkdir(parents=True)
        (d / 'name').write_text(label + '\n')
    for name, attrs in usb.items():
        d = tmp_path / 'bus' / 'usb' / 'devices' / name
        d.mkdir(parents=True)
        for key, value in attrs.items():
            (d / key).write_text(value + '\n')
    return str(tmp_path)


def test_find_zed_numeric_order_and_hid_serial(tmp_path):
    root = fake_sysfs(
        tmp_path,
        {'video0': 'Integrated Camera', 'video9': 'ZED 2i: ZED 2i', 'video10': 'ZED 2i: ZED 2i'},
        {'1-1': {'idVendor': '2b03', 'product': 'ZED 2i', 'serial': '0'},
         '1-2': {'idVendor': '2b03', 'product': 'ZED-2i HID INTERFACE', 'serial': '36243153'}})
    assert stereo.find_zed(root) == ('/dev/video9', '36243153')


def test_find_zed_nothing(tmp_path):
    root = fake_sysfs(tmp_path, {'video0': 'Integrated Camera'}, {})
    assert stereo.find_zed(root) == (None, None)


def test_calibration_dirs_prefers_repo_then_sdk_then_cache(tmp_path):
    ws = tmp_path / 'repo' / 'ros2_ws'
    dirs = stereo.calibration_dirs(env={'PIXI_PROJECT_ROOT': str(ws)}, cwd=str(ws))
    assert dirs[0] == str(tmp_path / 'repo' / 'configs' / 'zed')
    assert dirs[-2] == '/usr/local/zed/settings'
    assert dirs[-1].endswith('.local/share/zed/settings')
    assert len(dirs) == len(set(dirs))


def test_find_calibration(tmp_path):
    a, b = tmp_path / 'a', tmp_path / 'b'
    a.mkdir()
    b.mkdir()
    (b / 'SN1.conf').write_text('[STEREO]\n')
    assert stereo.find_calibration('1', [str(a), str(b)]) == str(b / 'SN1.conf')
    assert stereo.find_calibration('2', [str(a), str(b)]) is None


def test_zed_sdk_present(tmp_path):
    assert not stereo.zed_sdk_present(str(tmp_path))
    (tmp_path / 'etc').mkdir()
    (tmp_path / 'etc' / 'nv_tegra_release').write_text('# R36\n')
    assert stereo.zed_sdk_present(str(tmp_path))


def test_disparity_to_depth():
    disp = np.array([[0.0, 10.0, 100.0, 1.0]], dtype=np.float32)
    depth = stereo.disparity_to_depth(disp, fx_baseline=50.0, min_depth=0.3, max_depth=10.0)
    assert np.isnan(depth[0, 0])           # no match
    assert depth[0, 1] == pytest.approx(5.0)
    assert depth[0, 2] == pytest.approx(0.5)
    assert np.isnan(depth[0, 3])           # 50 m: beyond max_depth


def test_depth_to_cloud_axes_and_colour():
    P = np.array([[100.0, 0, 2.0, 0], [0, 100.0, 1.0, 0], [0, 0, 1, 0]])
    depth = np.full((3, 5), np.nan, dtype=np.float32)
    depth[1, 2] = 2.0   # principal point: straight ahead
    depth[0, 4] = 1.0   # right of and above centre
    bgr = np.zeros((3, 5, 3), dtype=np.uint8)
    bgr[1, 2] = (10, 20, 30)
    pts = stereo.depth_to_cloud(depth, bgr, P, step=1)
    assert pts.size == 2
    ahead = pts[np.argmax(pts['x'])]
    assert (ahead['x'], ahead['y'], ahead['z']) == pytest.approx((2.0, 0.0, 0.0))
    packed = struct.unpack('<I', struct.pack('<f', ahead['rgb']))[0]
    assert packed == (30 << 16) | (20 << 8) | 10
    side = pts[np.argmin(pts['x'])]
    # pixel u=4 > cx: right of centre -> negative y (left is +y); v=0 < cy: above -> positive z
    assert side['x'] == pytest.approx(1.0)
    assert side['y'] == pytest.approx(-0.02)
    assert side['z'] == pytest.approx(0.01)


def test_depth_to_cloud_step():
    depth = np.ones((8, 8), dtype=np.float32)
    bgr = np.zeros((8, 8, 3), dtype=np.uint8)
    assert stereo.depth_to_cloud(depth, bgr, np.eye(3, 4) * 100, step=4).size == 4


@needs_conf
@pytest.mark.parametrize('res,w,h', [('VGA', 672, 376), ('HD', 1280, 720)])
def test_team_calibration_loads(res, w, h):
    Kl, Dl, Kr, Dr, R, T = stereo.load_calibration(TEAM_CONF, res)
    assert Dl.size == 8  # ZED 2i: rational model from [LEFT_DISTO]
    assert 0.11 < abs(T[0]) < 0.13  # ZED 2i baseline is 120 mm
    assert Kl[0, 2] < w and Kl[1, 2] < h


@needs_conf
def test_rectified_synthetic_scene_gives_its_depth():
    """A textured plane 2 m away, as the rectified pair would see it, comes back at 2 m."""
    w, h = 672, 376
    rect = stereo.Rectifier(stereo.load_calibration(TEAM_CONF, 'VGA'), w, h, min_depth=0.3, max_depth=10.0)
    rng = np.random.default_rng(0)
    texture = cv2.resize(rng.integers(0, 255, (h // 4, w // 2), dtype=np.uint8), (w * 2, h),
                         interpolation=cv2.INTER_NEAREST)
    # Rectified views of a fronto-parallel plane at Z differ by a disparity of fx*B/Z pixels:
    # right[x] = left[x + d].
    d = int(round(rect.fx_baseline / 2.0))
    left = cv2.cvtColor(np.ascontiguousarray(texture[:, :w]), cv2.COLOR_GRAY2BGR)
    right = cv2.cvtColor(np.ascontiguousarray(texture[:, d:d + w]), cv2.COLOR_GRAY2BGR)
    depth = rect.depth(left, right)[:, rect.num_disp:]  # SGBM leaves the left margin empty
    valid = depth[np.isfinite(depth)]
    assert valid.size > 0.5 * depth.size
    assert np.median(valid) == pytest.approx(rect.fx_baseline / d, rel=0.02)
    assert np.median(valid) == pytest.approx(2.0, rel=0.05)


@needs_conf
def test_flip_keeps_baseline():
    w, h = 672, 376
    cal = stereo.load_calibration(TEAM_CONF, 'VGA')
    upright = stereo.Rectifier(cal, w, h, 0.3, 10.0, flip=False)
    flipped = stereo.Rectifier(cal, w, h, 0.3, 10.0, flip=True)
    assert flipped.baseline == pytest.approx(upright.baseline, rel=1e-6)


@needs_conf
def test_split_returns_rectified_pair_shapes():
    w, h = 672, 376
    rect = stereo.Rectifier(stereo.load_calibration(TEAM_CONF, 'VGA'), w, h, 0.3, 10.0)
    frame = np.zeros((h, 2 * w, 3), dtype=np.uint8)
    left, right = rect.split(frame)
    assert left.shape == right.shape == (h, w, 3)
