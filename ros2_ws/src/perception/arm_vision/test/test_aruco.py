"""arm_vision.aruco against synthetic RGB-D frames.

A marker is rendered at a known pose into a pinhole camera, with the matching depth image, so
every position and orientation has a ground truth.
"""

import cv2
import numpy as np
import pytest
from arm_vision.aruco import (
    ArucoDetector,
    Intrinsics,
    deproject,
    draw_markers,
    generate_marker,
    marker_center,
    marker_depth,
    parse_marker_sizes,
    rotation_to_quaternion,
)
from arm_vision.generate_markers import main as generate_main
from arm_vision.usb import camera_warnings, realsense_cameras

W, H = 640, 480
K = np.array([[615.0, 0.0, 322.0], [0.0, 615.0, 238.0], [0.0, 0.0, 1.0]])  # D435i-like colour
INTR = Intrinsics(K=K, D=np.zeros(5))
SIZE = 0.08  # m, black square
DICT = "DICT_5X5_50"


def rotation(roll=0.0, pitch=0.0, yaw=0.0):
    """Marker -> camera rotation of a marker facing the camera (its z toward the lens, i.e. 180
    degrees about x from the camera's axes), then tilted by roll, pitch, yaw."""
    facing = cv2.Rodrigues(np.array([np.pi, 0.0, 0.0]))[0]
    return cv2.Rodrigues(np.array([roll, 0.0, 0.0]))[0] @ \
        cv2.Rodrigues(np.array([0.0, pitch, 0.0]))[0] @ \
        cv2.Rodrigues(np.array([0.0, 0.0, yaw]))[0] @ facing


def render(marker_id=7, t=(0.05, -0.02, 0.6), R=None, size=SIZE, background=140):
    """Colour (BGR) + depth (m) of a printed marker at pose (R, t) in front of the camera. The
    marker sits on an infinite plane (a wall), so every pixel has depth."""
    R = rotation(np.radians(20), np.radians(-15), np.radians(10)) if R is None else R
    t = np.asarray(t, dtype=float)
    side_px = 300
    tag = generate_marker(DICT, marker_id, side_px)            # marker + white quiet zone
    full = tag.shape[0]
    half = size * full / side_px / 2                            # half-width incl. the margin, m
    obj = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
    img_pts, _ = cv2.projectPoints(obj, cv2.Rodrigues(R)[0], t, K, np.zeros(5))
    src = np.array([[-0.5, -0.5], [full - 0.5, -0.5], [full - 0.5, full - 0.5],
                    [-0.5, full - 0.5]], dtype=np.float32)
    Hm = cv2.getPerspectiveTransform(src, img_pts.reshape(4, 2).astype(np.float32))
    gray = cv2.warpPerspective(tag, Hm, (W, H), flags=cv2.INTER_LINEAR,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=background)
    color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    n = R[:, 2]
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)])
    depth = (n @ t) / np.tensordot(n, rays, axes=1)              # z of the plane along each ray
    return color, depth.astype(np.float32), R, t


def angle_between(q1, q2):
    return np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(q1, q2))))))


@pytest.fixture
def detector():
    return ArucoDetector(dictionary=DICT, marker_size=SIZE)


def test_depth_position_and_pnp_orientation(detector):
    color, depth, R, t = render()
    markers = detector.process(color, INTR, depth)
    assert [m.id for m in markers] == [7]
    m = markers[0]
    assert m.position_source == "depth"
    assert np.linalg.norm(m.position - t) < 0.005          # 5 mm at 0.6 m
    assert abs(m.depth - t[2]) < 0.005
    assert m.distance == pytest.approx(np.linalg.norm(t), abs=0.005)
    assert angle_between(m.quaternion, rotation_to_quaternion(R)) < 3.0
    assert m.reprojection_error < 1.0
    assert np.linalg.norm(m.tvec.ravel() - t) < 0.01        # PnP agrees with depth


def test_no_depth_falls_back_to_pnp(detector):
    color, _, _, t = render()
    m = detector.process(color, INTR, None)[0]
    assert m.position_source == "pnp"
    assert np.isnan(m.depth)
    assert np.linalg.norm(m.position - t) < 0.01


def test_per_id_marker_size():
    """marker_sizes overrides marker_size for its ids (the keyboard's smaller markers)."""
    color, _, _, t = render(size=SIZE / 2)
    wrong = ArucoDetector(DICT, marker_size=SIZE).process(color, INTR, None)[0]
    right = ArucoDetector(DICT, marker_size=SIZE, marker_sizes={7: SIZE / 2}).process(
        color, INTR, None)[0]
    assert np.linalg.norm(wrong.position - t) > 0.2   # PnP twice as far
    assert np.linalg.norm(right.position - t) < 0.01


def test_parse_marker_sizes():
    assert parse_marker_sizes(["10:0.03", " ", "4:0.1"]) == {10: 0.03, 4: 0.1}
    with pytest.raises(ValueError):
        parse_marker_sizes(["10"])


def test_mostly_missing_depth_falls_back_to_pnp(detector):
    color, depth, _, t = render()
    depth[::, ::] = 0.0
    depth[::7, ::7] = t[2]  # ~2 % valid: below min_valid_depth_fraction
    m = detector.process(color, INTR, depth)[0]
    assert m.position_source == "pnp"


def test_depth_holes_and_outliers_dont_move_the_median(detector):
    color, depth, _, t = render()
    rng = np.random.default_rng(0)
    holes = rng.random(depth.shape) < 0.4
    depth[holes] = 0.0                                       # 40 % no data
    depth[rng.random(depth.shape) < 0.05] = 3.0              # 5 % flying pixels
    m = detector.process(color, INTR, depth)[0]
    assert m.position_source == "depth"
    assert abs(m.depth - t[2]) < 0.01


def test_position_source_depth_only_without_depth_gives_no_position():
    color, _, _, _ = render()
    det = ArucoDetector(dictionary=DICT, marker_size=SIZE, position_source="depth")
    m = det.process(color, INTR, None)[0]
    assert m.position is None and m.quaternion is not None


def test_unknown_marker_size_depth_position_no_orientation():
    color, depth, _, t = render()
    det = ArucoDetector(dictionary=DICT, marker_size=0.0)
    m = det.process(color, INTR, depth)[0]
    assert m.quaternion is None and np.linalg.norm(m.position - t) < 0.005


def test_without_intrinsics_only_2d_and_depth(detector):
    color, depth, _, t = render()
    m = detector.process(color, None, depth)[0]
    assert m.position is None and abs(m.depth - t[2]) < 0.005


def test_several_markers_sorted_by_id(detector):
    a, da, _, _ = render(marker_id=12, t=(-0.12, 0.0, 0.7), R=rotation())
    b, db, _, _ = render(marker_id=3, t=(0.12, 0.0, 0.7), R=rotation())
    color = np.where(np.arange(W)[None, :, None] < W // 2, a, b).astype(np.uint8)
    depth = np.where(np.arange(W)[None, :] < W // 2, da, db)
    markers = detector.process(color, INTR, depth)
    assert [m.id for m in markers] == [3, 12]
    assert markers[0].position[0] > 0 > markers[1].position[0]


def test_wrong_dictionary_finds_nothing():
    color, depth, _, _ = render()
    assert ArucoDetector(dictionary="DICT_4X4_50").process(color, INTR, depth) == []


def test_deproject_undoes_lens_distortion():
    D = np.array([0.12, -0.25, 0.001, -0.002, 0.05])
    p = np.array([0.35, -0.22, 0.9])  # well off-centre, where distortion is large
    uv, _ = cv2.projectPoints(p.reshape(1, 3), np.zeros(3), np.zeros(3), K, D)
    uv = uv.ravel()
    assert np.allclose(deproject(uv, p[2], Intrinsics(K=K, D=D)), p, atol=1e-4)
    assert not np.allclose(deproject(uv, p[2], INTR), p, atol=1e-3)  # ignoring D is wrong


def test_marker_center_is_diagonal_intersection():
    quad = np.array([[0.0, 0.0], [10.0, 0.0], [8.0, 6.0], [2.0, 6.0]])  # trapezoid
    c = marker_center(quad)
    # on both diagonals
    for a, b in ((quad[0], quad[2]), (quad[1], quad[3])):
        (x1, y1), (x2, y2) = b - a, c - a
        assert abs(x1 * y2 - y1 * x2) < 1e-9
    assert not np.allclose(c, quad.mean(axis=0))


def test_marker_depth_tiny_marker_uses_centre():
    depth = np.full((20, 20), 1.5, dtype=np.float32)
    quad = np.array([[10.0, 10.0], [10.6, 10.0], [10.6, 10.6], [10.0, 10.6]])
    assert marker_depth(depth, quad) == pytest.approx(1.5)


def test_marker_depth_outside_image_is_nan():
    depth = np.ones((10, 10), dtype=np.float32)
    quad = np.array([[20.0, 20.0], [30.0, 20.0], [30.0, 30.0], [20.0, 30.0]])
    assert np.isnan(marker_depth(depth, quad))


def test_deproject_inverts_projection():
    p = np.array([0.1, -0.2, 1.3])
    uv = (K @ p)[:2] / p[2]
    assert np.allclose(deproject(uv, p[2], INTR), p, atol=1e-9)


@pytest.mark.parametrize("seed", range(5))
def test_rotation_to_quaternion_matches_rodrigues(seed):
    rvec = np.random.default_rng(seed).normal(size=3)
    R = cv2.Rodrigues(rvec)[0]
    x, y, z, w = rotation_to_quaternion(R)
    Rq = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    assert np.allclose(R, Rq, atol=1e-9) and w >= 0


def test_intrinsics_from_camera_info_odd_distortion():
    intr = Intrinsics.from_camera_info(K.ravel(), [0.1, 0.2, 0.3])  # 3 coefficients: unusable
    assert intr.valid and np.allclose(intr.D, 0)
    assert not Intrinsics.from_camera_info(np.zeros(9), []).valid


def test_draw_markers_keeps_input(detector):
    color, depth, _, _ = render()
    original = color.copy()
    markers = detector.process(color, INTR, depth)
    out = draw_markers(color, markers, INTR, SIZE)
    assert np.array_equal(color, original) and not np.array_equal(out, color)


def test_bad_parameters():
    with pytest.raises(ValueError):
        ArucoDetector(dictionary="DICT_NOPE")
    with pytest.raises(ValueError):
        ArucoDetector(position_source="lidar")
    with pytest.raises(ValueError):
        generate_marker(DICT, 50)  # DICT_5X5_50 has ids 0-49


def test_generated_png_has_dpi_and_detects(tmp_path):
    generate_main(["--ids", "4", "--size-mm", "50", "--dpi", "300", "--out", str(tmp_path)])
    path = tmp_path / "DICT_5X5_50_4_50mm.png"
    data = path.read_bytes()
    assert b"pHYs" in data[:64]
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    assert img is not None
    found = ArucoDetector(dictionary=DICT).detect(cv2.copyMakeBorder(
        img, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255))
    assert [m.id for m in found] == [4]
    side = round(50 / 25.4 * 300)
    assert img.shape[0] == side + 2 * (side // 7)


def fake_usb(tmp_path, name, vendor, product, speed, serial="123"):
    d = tmp_path / name
    d.mkdir()
    for k, v in (("idVendor", vendor), ("idProduct", product), ("speed", speed),
                 ("serial", serial)):
        (d / k).write_text(v + "\n")


def test_usb_scan(tmp_path):
    fake_usb(tmp_path, "2-1", "8086", "0b3a", "5000", "838212070123")
    fake_usb(tmp_path, "1-2", "8086", "0b07", "480")
    fake_usb(tmp_path, "1-3", "8086", "0adb", "5000")
    fake_usb(tmp_path, "1-4", "2b03", "f582", "5000")   # ZED 2i: not a RealSense
    fake_usb(tmp_path, "1-5", "8086", "0aaa", "5000")   # some other Intel device
    cams = {c.port: c for c in realsense_cameras(str(tmp_path))}
    assert set(cams) == {"2-1", "1-2", "1-3"}
    assert cams["2-1"].model == "D435i" and cams["2-1"].usb3
    assert cams["2-1"].serial == "838212070123"
    warnings = camera_warnings(list(cams.values()))
    assert len(warnings) == 2
    assert any("USB 2" in w for w in warnings) and any("recovery" in w for w in warnings)
    assert realsense_cameras(str(tmp_path / "missing")) == []
