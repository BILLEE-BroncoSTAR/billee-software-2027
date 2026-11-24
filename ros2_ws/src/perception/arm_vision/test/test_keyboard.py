"""arm_vision.keyboard: layout parsing, text -> keys, and the keyboard pose from synthetic RGB-D
frames of the markers on a keyboard, against ground truth."""

import os

import cv2
import numpy as np
import pytest
from arm_vision.aruco import ArucoDetector, Intrinsics, generate_marker
from arm_vision.keyboard import (
    KEY_UNIT,
    KeyboardLayout,
    KeyboardLocalizer,
    MarkerObservation,
    PoseFilter,
    keys_for_text,
    parse_key_rows,
    quaternion_to_rotation,
    rotation_to_quaternion,
)

W, H = 640, 480
K = np.array([[615.0, 0.0, 322.0], [0.0, 615.0, 238.0], [0.0, 0.0, 1.0]])
INTR = Intrinsics(K=K, D=np.zeros(5))
DICT = "DICT_5X5_50"
LAYOUT_FILE = os.path.join(os.path.dirname(__file__), "..", "config", "keyboard_layout.yaml")


@pytest.fixture(scope="module")
def layout():
    return KeyboardLayout.from_yaml(LAYOUT_FILE)


def rot(axis, deg):
    v = np.zeros(3)
    v["xyz".index(axis)] = np.radians(deg)
    return cv2.Rodrigues(v)[0]


def looking_down(tilt_x=25.0, tilt_y=-10.0, yaw=8.0):
    """keyboard -> camera rotation for a camera above the keyboard looking down at it (as from
    the end effector), tilted toward the typist and sideways."""
    down = np.diag([1.0, -1.0, -1.0])  # keyboard x -> image right, y -> image up, z -> toward lens
    return rot("x", tilt_x) @ rot("y", tilt_y) @ rot("z", yaw) @ down


def pose_for(layout, R, distance=0.45):
    center = np.array([layout.size[0] / 2, layout.size[1] / 2, 0.0])
    t = np.array([0.0, 0.0, distance]) - R @ center
    return R, t


def render(layout, R, t, ids=None, mm_per_px=0.5, background=150):
    """BGR + depth image of the markers' plane (all layout markers share one z here) with the
    markers printed on it, seen by the camera at keyboard -> camera pose (R, t)."""
    ms = [m for m in layout.markers.values() if ids is None or m.id in ids]
    z = ms[0].position[2]
    assert all(abs(m.position[2] - z) < 1e-9 for m in layout.markers.values())
    lo = np.min([m.position[:2] - m.size for m in layout.markers.values()], axis=0) - 0.02
    hi = np.max([m.position[:2] + m.size for m in layout.markers.values()], axis=0) + 0.02
    scale = 1000.0 / mm_per_px  # px per m
    pw, ph = (np.ceil((hi - lo) * scale)).astype(int)
    plate = np.full((ph, pw), 235, np.uint8)  # a light keyboard case
    for m in ms:
        side = round(m.size * scale)
        tag = generate_marker(DICT, m.id, side)
        margin = (tag.shape[0] - side) // 2
        tag = np.rot90(tag, k=round(np.degrees(m.yaw) / 90)) if m.yaw else tag
        cx = (m.position[0] - lo[0]) * scale
        cy = (hi[1] - m.position[1]) * scale  # plate rows run toward the typist (-y)
        x0, y0 = round(cx - side / 2) - margin, round(cy - side / 2) - margin
        plate[y0:y0 + tag.shape[0], x0:x0 + tag.shape[1]] = tag
    # plate pixel corners -> keyboard frame -> image
    corners_px = np.array([[0, 0], [pw, 0], [pw, ph], [0, ph]], np.float32) - 0.5
    corners_kb = np.array([[lo[0] + u / scale, hi[1] - v / scale, z] for u, v in corners_px + 0.5])
    img, _ = cv2.projectPoints(corners_kb, cv2.Rodrigues(R)[0], t, K, np.zeros(5))
    Hm = cv2.getPerspectiveTransform(corners_px, img.reshape(4, 2).astype(np.float32))
    gray = cv2.warpPerspective(plate, Hm, (W, H), flags=cv2.INTER_AREA,
                               borderMode=cv2.BORDER_CONSTANT, borderValue=background)
    n = R[:, 2]
    p0 = R @ np.array([0, 0, z]) + t
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)])
    depth = (n @ p0) / np.tensordot(n, rays, axes=1)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), depth


def observe(layout, color, depth, use_depth=True):
    det = ArucoDetector(DICT, marker_sizes={i: m.size for i, m in layout.markers.items()})
    markers = det.process(color, INTR, depth if use_depth else None)
    return [MarkerObservation(id=m.id, corners=m.corners,
                              depth_position=m.position if m.position_source == "depth"
                              else None) for m in markers]


def angle_between(Ra, Rb):
    return np.degrees(np.arccos(np.clip((np.trace(Ra.T @ Rb) - 1) / 2, -1, 1)))


# --- layout ---------------------------------------------------------------------------------

def test_layout_keys(layout):
    assert len(layout.markers) == 4
    # ANSI: Q is 1.5 u in on the third row from the front + F-row gap; A 1.75 u in.
    q, a, space = layout.key("Q"), layout.key("A"), layout.key("Space")
    np.testing.assert_allclose(q.center[:2], [2.0 * KEY_UNIT, 3.5 * KEY_UNIT])
    np.testing.assert_allclose(a.center[:2], [2.25 * KEY_UNIT, 2.5 * KEY_UNIT])
    np.testing.assert_allclose(space.center[:2], [(3.75 + 3.125) * KEY_UNIT, 0.5 * KEY_UNIT])
    np.testing.assert_allclose(layout.key("Esc").center[1], 6.0 * KEY_UNIT)  # 0.5 u gap
    np.testing.assert_allclose(layout.size, [15 * KEY_UNIT, 6.5 * KEY_UNIT])
    assert layout.key("\\").width == pytest.approx(1.5 * KEY_UNIT * 0.95)


def test_rows_errors():
    with pytest.raises(ValueError, match="twice"):
        parse_key_rows(["Shift A Shift"], KEY_UNIT)
    keys, size = parse_key_rows([{"keys": "_:2 X:1.5", "x": 1}], 0.01)
    np.testing.assert_allclose(keys["X"].center[:2], [0.0375, 0.005])
    np.testing.assert_allclose(size, [0.045, 0.01])
    with pytest.raises(KeyError):
        KeyboardLayout.from_dict({"markers": [{"id": 1, "x": 0, "y": 0}],
                                  "rows": ["A"]}).key("B")
    with pytest.raises(ValueError, match="no markers"):
        KeyboardLayout.from_dict({"rows": ["A"]})


def test_keys_for_text(layout):
    assert keys_for_text("Hi, URC!\n", layout) == [
        ("H", "Shift"), ("I",), (",",), ("Space",), ("U", "Shift"), ("R", "Shift"),
        ("C", "Shift"), ("1", "Shift"), ("Enter",)]
    assert keys_for_text('a"_', layout) == [("A",), ("'", "Shift"), ("-", "Shift")]
    with pytest.raises(KeyError, match="é"):
        keys_for_text("é", layout)


def test_marker_corners_yaw():
    lay = KeyboardLayout.from_dict({"markers": [{"id": 3, "x": 0.1, "y": 0.2, "size": 0.02,
                                                 "yaw": 90}], "rows": []})
    c = lay.markers[3].corners()
    # yaw 90: the marker's top-left corner (-s/2, +s/2) turns to (-s/2, -s/2)
    np.testing.assert_allclose(c[0], [0.09, 0.19, 0.0], atol=1e-12)


# --- pose -----------------------------------------------------------------------------------

@pytest.mark.parametrize("use_depth", [True, False])
def test_pose_all_markers(layout, use_depth):
    R, t = pose_for(layout, looking_down())
    color, depth = render(layout, R, t)
    obs = observe(layout, color, depth, use_depth)
    assert sorted(o.id for o in obs) == [10, 11, 12, 13]
    est = KeyboardLocalizer(layout).estimate(obs, INTR)
    assert est is not None
    assert est.marker_ids == [10, 11, 12, 13]
    assert est.reprojection_error < 1.0
    assert angle_between(est.R, R) < 0.5
    np.testing.assert_allclose(est.t, t, atol=0.004)
    assert (est.depth_correction is not None) == use_depth
    # every key within a few mm of where it really is
    centers = np.array([k.center for k in layout.keys.values()])
    err = np.linalg.norm(est.to_camera(centers) - (centers @ R.T + t), axis=1)
    assert err.max() < 0.005


def test_pose_one_marker(layout):
    """One marker in view still places the keyboard, less precisely."""
    R, t = pose_for(layout, looking_down(15, 5, -5), distance=0.4)
    color, depth = render(layout, R, t, ids={12})
    obs = observe(layout, color, depth)
    assert [o.id for o in obs] == [12]
    est = KeyboardLocalizer(layout).estimate(obs, INTR)
    assert est is not None and est.marker_ids == [12]
    assert angle_between(est.R, R) < 3.0
    centers = np.array([k.center for k in layout.keys.values()])
    err = np.linalg.norm(est.to_camera(centers) - (centers @ R.T + t), axis=1)
    assert err.max() < 0.02  # the far keys are 30 cm from the marker
    assert KeyboardLocalizer(layout, min_markers=2).estimate(obs, INTR) is None


@pytest.mark.parametrize("ids", [None, {12}])
def test_wrong_marker_size(layout, ids):
    """Markers printed 3 % small (printer scaling). With several markers the spacing in the
    layout sets the scale, so it barely matters; with one, PnP alone is ~3 % too close and the
    depth correction fixes it."""
    small = KeyboardLayout.from_yaml(LAYOUT_FILE)
    for m in small.markers.values():
        m.size *= 0.97
    R, t = pose_for(layout, looking_down(), distance=0.5)
    color, depth = render(small, R, t, ids=ids)  # the real (small) markers
    obs = observe(small, color, depth)
    center = np.array([*layout.size / 2, 0.0])
    truth = R @ center + t

    def error(**kw):
        est = KeyboardLocalizer(layout, max_reprojection_error=10, **kw).estimate(obs, INTR)
        return np.linalg.norm(est.to_camera(center) - truth)

    if ids is None:
        assert error(use_depth=False) < 0.003
        assert error() < 0.003
    else:
        assert error(use_depth=False) > 0.008
        assert error() < 0.006


def test_marker_size_override(layout):
    """marker_sizes:= at launch: the keyboard's markers printed at 40 mm instead of the layout's
    30 mm. With the layout's sizes every corner is off and the fit is rejected; with the
    override (what keyboard_node applies) it's as good as a matching layout."""
    big = layout.with_marker_sizes({i: 0.04 for i in (10, 11, 12, 13)} | {99: 0.1})
    assert all(m.size == 0.04 for m in big.markers.values()) and 99 not in big.markers
    assert layout.markers[10].size == 0.03  # the original is untouched
    assert big.keys == layout.keys
    R, t = pose_for(layout, looking_down())
    color, depth = render(big, R, t)  # the real (40 mm) markers
    obs = observe(big, color, depth)
    wrong = KeyboardLocalizer(layout)
    assert wrong.estimate(obs, INTR) is None
    assert "reprojection error" in wrong.last_rejection
    est = KeyboardLocalizer(big).estimate(obs, INTR)
    assert est is not None and est.reprojection_error < 1.0
    np.testing.assert_allclose(est.t, t, atol=0.004)


def test_rejects_moved_marker(layout):
    """A marker that isn't where the layout says (moved 4 cm) is caught by the reprojection
    error instead of silently skewing the keyboard."""
    moved = KeyboardLayout.from_yaml(LAYOUT_FILE)
    moved.markers[13].position = moved.markers[13].position + [0.0, 0.04, 0.0]
    R, t = pose_for(layout, looking_down())
    color, depth = render(moved, R, t)
    obs = observe(layout, color, depth)
    loc = KeyboardLocalizer(layout, max_reprojection_error=2.0)
    assert loc.estimate(obs, INTR) is None
    assert "reprojection error" in loc.last_rejection


def test_ignores_other_ids(layout):
    loc = KeyboardLocalizer(layout)
    obs = [MarkerObservation(id=3, corners=np.array([[0, 0], [10, 0], [10, 10], [0, 10.0]]))]
    assert loc.estimate(obs, INTR) is None
    assert "0 layout marker" in loc.last_rejection


# --- filter ---------------------------------------------------------------------------------

def test_filter_smooths_and_resets():
    f = PoseFilter(alpha=0.5, reset_distance=0.05)
    q = rotation_to_quaternion(np.eye(3))
    f.update([0, 0, 0], q)
    p, _ = f.update([0.01, 0, 0], q)
    np.testing.assert_allclose(p, [0.005, 0, 0])
    p, _ = f.update([0.5, 0, 0], q)  # jumped: start over
    np.testing.assert_allclose(p, [0.5, 0, 0])
    assert f.samples == 1
    _, qq = f.update([0.5, 0, 0], rotation_to_quaternion(rot("z", 4)))
    assert angle_between(quaternion_to_rotation(qq), rot("z", 2)) < 0.01


# --- keyboard_node messages -----------------------------------------------------------------

def test_keyboard_markers_with_real_messages(layout):
    """keyboard_node's MarkerArray, built with the real ROS message classes: they reject an int
    in a float field (AssertionError), which the stubbed messages it was first written against
    didn't, and the node died on its first visualization tick."""
    pytest.importorskip("visualization_msgs.msg")
    from arm_vision.keyboard_node import keyboard_markers
    from builtin_interfaces.msg import Time

    out = keyboard_markers(layout, "keyboard", Time(sec=1))
    by_ns = {}
    for m in out.markers:
        by_ns.setdefault(m.ns, []).append(m)
    assert len(by_ns["keyboard_keys"]) == len(by_ns["keyboard_labels"]) == len(layout.keys)
    assert len(by_ns["keyboard_markers"]) == len(layout.markers)
    outline = by_ns["keyboard_outline"][0]
    assert [(p.x, p.y) for p in outline.points][:3] == [
        (0.0, 0.0), (layout.size[0], 0.0), (layout.size[0], layout.size[1])]
    assert all(m.header.frame_id == "keyboard" for m in out.markers)
