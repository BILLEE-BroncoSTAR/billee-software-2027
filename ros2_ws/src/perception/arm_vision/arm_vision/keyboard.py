"""Keyboard pose from ArUco markers stuck on it, and where each key is. No ROS (unit-tested).

A layout file (config/keyboard_layout.yaml) describes one keyboard in its own frame:

    keyboard frame   origin at the front-left corner of the key field (the outer corner of the
                     bottom-left key, nearest the typist), x along the rows to the right, y away
                     from the typist (toward the number row), z up, out of the key tops; z = 0 is
                     the top of the keys.
    markers          ArUco ids on the case, each with its centre (x, y, z), its yaw about z and
                     its printed size. An upright marker (yaw 0) reads normally to the typist: its
                     top edge is toward the far side of the keyboard.
    keys             rows of key labels and widths, in key units (1 u = 19.05 mm on most
                     keyboards), so the key centres come out of the row layout.

The pose (keyboard -> camera) comes from solvePnP over the corners of every layout marker in
view, as one rigid object, so several markers give a steadier orientation than any one marker.
The distance along the camera's line of sight is PnP's weak axis (a 1 % scale error is 1 % of the
range); when markers also have depth positions (aruco_node, position_source "depth"), the
translation is shifted so the markers' predicted centres meet the measured ones on average.
"""

from __future__ import annotations

from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field

import cv2
import numpy as np
import yaml

from arm_vision.aruco import Intrinsics, marker_object_points, rotation_to_quaternion

KEY_UNIT = 0.01905  # m, the usual 0.75 in key pitch


@dataclass
class LayoutMarker:
    id: int
    size: float               # m, black square
    position: np.ndarray      # (3,) centre in the keyboard frame
    yaw: float = 0.0          # rad about the keyboard's z

    def corners(self) -> np.ndarray:
        """(4, 3) corners in the keyboard frame, in OpenCV's detection order (TL, TR, BR, BL)."""
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        return marker_object_points(self.size) @ R.T + self.position


@dataclass
class Key:
    label: str
    center: np.ndarray        # (3,) top centre in the keyboard frame
    width: float              # m, along x
    height: float             # m, along y


@dataclass
class KeyboardLayout:
    name: str
    markers: dict[int, LayoutMarker]
    keys: dict[str, Key]
    key_unit: float = KEY_UNIT
    size: np.ndarray = field(default_factory=lambda: np.zeros(2))  # key field (x, y), m

    @classmethod
    def from_yaml(cls, path: str) -> KeyboardLayout:
        with open(path) as f:
            return cls.from_dict(yaml.safe_load(f))

    @classmethod
    def from_dict(cls, data: dict) -> KeyboardLayout:
        unit = float(data.get("key_unit", KEY_UNIT))
        default_size = float(data.get("marker_size", 0.03))
        markers = {}
        for m in data.get("markers", []):
            mid = int(m["id"])
            if mid in markers:
                raise ValueError(f"marker id {mid} listed twice")
            markers[mid] = LayoutMarker(
                id=mid, size=float(m.get("size", default_size)),
                position=np.array([float(m["x"]), float(m["y"]), float(m.get("z", 0.0))]),
                yaw=np.radians(float(m.get("yaw", 0.0))))
        if not markers:
            raise ValueError("the layout has no markers")
        keys, extent = parse_key_rows(data.get("rows", []), unit,
                                      float(data.get("key_size", 1.0)))
        return cls(name=str(data.get("name", "keyboard")), markers=markers, keys=keys,
                   key_unit=unit, size=extent)

    def key(self, label: str) -> Key:
        try:
            return self.keys[label]
        except KeyError:
            raise KeyError(f"no key {label!r} in layout {self.name!r}") from None


def parse_key_rows(rows: Sequence, unit: float, key_size: float = 1.0
                   ) -> tuple[dict[str, Key], np.ndarray]:
    """Rows from the far side (listed first) to the typist (listed last) -> keys by label.

    Each row is a string of space-separated keys, or {keys: "...", gap: <u>, x: <u>}:
      "Q"          a 1 u key labelled Q
      "Tab:1.5"    1.5 u wide
      "_:0.5"      0.5 u of empty space (no key)
    gap: extra space (u) between this row and the next one toward the typist, e.g. between the
    F-key row and the number row. x: where the row starts (u from the left edge).
    Each key is one unit deep; key_size (u) is the keycap's top size relative to its pitch (only
    for drawing). Labels must be unique: name the second Shift "RShift".
    """
    parsed = []
    for row in rows:
        if isinstance(row, str):
            row = {"keys": row}
        parsed.append((str(row["keys"]).split(), float(row.get("gap", 0.0)),
                       float(row.get("x", 0.0))))
    # y of each row centre, counted from the front (the last row) upward.
    depth_u = len(parsed) + sum(gap for _, gap, _ in parsed[:-1])
    keys: dict[str, Key] = {}
    y_top = depth_u  # far edge of the first row, in u
    width_u = 0.0
    for tokens, gap, x0 in parsed:
        y_c = (y_top - 0.5) * unit
        x = x0
        for token in tokens:
            label, width = split_token(token)
            if label != "_":
                if label in keys:
                    raise ValueError(f"key {label!r} appears twice; give one another label")
                keys[label] = Key(label=label,
                                  center=np.array([(x + width / 2) * unit, y_c, 0.0]),
                                  width=width * unit * key_size, height=unit * key_size)
            x += width
        width_u = max(width_u, x)
        y_top -= 1.0 + gap
    return keys, np.array([width_u * unit, depth_u * unit])


def split_token(token: str) -> tuple[str, float]:
    label, sep, width = token.rpartition(":")
    if sep and label:
        try:
            return label, float(width)
        except ValueError:
            pass
    return token, 1.0


# --- text -> keys ---------------------------------------------------------------------------

SHIFTED = {
    "~": "`", "!": "1", "@": "2", "#": "3", "$": "4", "%": "5", "^": "6", "&": "7", "*": "8",
    "(": "9", ")": "0", "_": "-", "+": "=", "{": "[", "}": "]", "|": "\\", ":": ";", '"': "'",
    "<": ",", ">": ".", "?": "/",
}
NAMED = {" ": "Space", "\n": "Enter", "\t": "Tab", "\b": "Backspace"}


def keys_for_text(text: str, layout: KeyboardLayout, shift: str = "Shift"
                  ) -> list[tuple[str, ...]]:
    """What to press for each character: [("H", "Shift") ...] as (key, modifier...) tuples, the
    key first. Letters are the layout's capital labels; Shift is held for capitals and symbols.
    Raises KeyError for a character the layout can't type."""
    out = []
    for ch in text:
        if ch in NAMED:
            chord = (NAMED[ch],)
        elif ch.isalpha() and ch.upper() in layout.keys:
            chord = (ch.upper(), shift) if ch.isupper() else (ch.upper(),)
        elif ch in SHIFTED:
            chord = (SHIFTED[ch], shift)
        else:
            chord = (ch,)
        for label in chord:
            if label not in layout.keys:
                raise KeyError(f"can't type {ch!r}: no key {label!r} in layout {layout.name!r}")
        out.append(chord)
    return out


# --- pose -----------------------------------------------------------------------------------

@dataclass
class MarkerObservation:
    """One marker in one image: its corners (px, OpenCV order) and, if it had valid depth, its
    centre measured in the camera frame (m)."""

    id: int
    corners: np.ndarray
    depth_position: np.ndarray | None = None


@dataclass
class KeyboardPose:
    R: np.ndarray                     # keyboard -> camera rotation
    t: np.ndarray                     # (3,) keyboard origin in the camera frame
    marker_ids: list[int]
    reprojection_error: float         # RMS px over every corner used
    depth_correction: np.ndarray | None = None  # (3,) m applied to the PnP translation

    @property
    def quaternion(self) -> np.ndarray:
        return rotation_to_quaternion(self.R)

    def matrix(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = self.R, self.t
        return T

    def to_camera(self, points: np.ndarray) -> np.ndarray:
        return np.asarray(points, dtype=float) @ self.R.T + self.t


class KeyboardLocalizer:
    """layout markers in view -> KeyboardPose, or None.

    max_reprojection_error: px RMS; above it the fit is rejected (a marker moved, a wrong id).
    use_depth: correct the translation from marker depths.
    max_depth_correction: m; a bigger disagreement between PnP and depth means one of them is
    wrong (marker size or layout mismeasured, depth on a glossy marker): depth is then ignored.
    """

    def __init__(self, layout: KeyboardLayout, min_markers: int = 1,
                 max_reprojection_error: float = 4.0, use_depth: bool = True,
                 max_depth_correction: float = 0.05):
        self.layout = layout
        self.min_markers = max(1, int(min_markers))
        self.max_reprojection_error = float(max_reprojection_error)
        self.use_depth = bool(use_depth)
        self.max_depth_correction = float(max_depth_correction)
        self.last_rejection = ""

    def estimate(self, observations: Iterable[MarkerObservation],
                 intrinsics: Intrinsics) -> KeyboardPose | None:
        obs = [o for o in observations if o.id in self.layout.markers]
        obs = list({o.id: o for o in obs}.values())  # one per id
        self.last_rejection = ""
        if len(obs) < self.min_markers or not obs:
            self.last_rejection = (f"{len(obs)} layout marker(s) in view, need "
                                   f"{self.min_markers}")
            return None
        obj = np.concatenate([self.layout.markers[o.id].corners() for o in obs])
        img = np.concatenate([np.asarray(o.corners, dtype=np.float64).reshape(4, 2)
                              for o in obs])
        rvec, tvec = self._initial_guess(obs, intrinsics)
        if rvec is None:
            self.last_rejection = "solvePnP failed"
            return None
        if len(obs) > 1:
            ok, rvec, tvec = cv2.solvePnP(obj, img.reshape(-1, 1, 2), intrinsics.K, intrinsics.D,
                                          rvec, tvec, useExtrinsicGuess=True,
                                          flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                self.last_rejection = "solvePnP failed"
                return None
        error = reprojection_error(obj, img, rvec, tvec, intrinsics)
        if error > self.max_reprojection_error:
            self.last_rejection = (f"reprojection error {error:.1f} px > "
                                   f"{self.max_reprojection_error:.1f} (layout or marker sizes "
                                   "wrong, or a marker moved)")
            return None
        R = cv2.Rodrigues(rvec)[0]
        t = tvec.ravel().astype(float)
        if t[2] <= 0:
            self.last_rejection = "keyboard behind the camera"
            return None
        pose = KeyboardPose(R=R, t=t, marker_ids=sorted(o.id for o in obs),
                            reprojection_error=error)
        if self.use_depth:
            self._depth_correct(pose, obs)
        return pose

    def _initial_guess(self, obs, intrinsics):
        """Pose from the largest marker in the image alone (IPPE_SQUARE: no planar ambiguity
        flip for a single square), moved from that marker to the keyboard origin."""
        best = max(obs, key=lambda o: cv2.contourArea(
            np.asarray(o.corners, dtype=np.float32).reshape(4, 2)))
        lm = self.layout.markers[best.id]
        try:
            ok, rvec, tvec = cv2.solvePnP(
                marker_object_points(lm.size), np.asarray(best.corners, dtype=np.float64)
                .reshape(4, 1, 2), intrinsics.K, intrinsics.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        except cv2.error:
            return None, None
        if not ok:
            return None, None
        R_cm = cv2.Rodrigues(rvec)[0]                  # marker -> camera
        c, s = np.cos(lm.yaw), np.sin(lm.yaw)
        R_km = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])  # marker -> keyboard
        R_ck = R_cm @ R_km.T                           # keyboard -> camera
        t_ck = tvec.ravel() - R_ck @ lm.position
        return cv2.Rodrigues(R_ck)[0], t_ck.reshape(3, 1)

    def _depth_correct(self, pose: KeyboardPose, obs):
        measured = [(self.layout.markers[o.id].position, np.asarray(o.depth_position, float))
                    for o in obs if o.depth_position is not None
                    and np.all(np.isfinite(o.depth_position))]
        if not measured:
            return
        predicted = pose.to_camera(np.array([p for p, _ in measured]))
        shift = np.mean(np.array([m for _, m in measured]) - predicted, axis=0)
        if np.linalg.norm(shift) <= self.max_depth_correction:
            pose.t = pose.t + shift
            pose.depth_correction = shift


def reprojection_error(obj, img, rvec, tvec, intrinsics: Intrinsics) -> float:
    projected, _ = cv2.projectPoints(obj, rvec, tvec, intrinsics.K, intrinsics.D)
    diff = projected.reshape(-1, 2) - np.asarray(img).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(diff ** 2, axis=1))))


# --- smoothing ------------------------------------------------------------------------------

def quaternion_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1, dot = -q1, -dot
    if dot > 0.9995:
        q = q0 + t * (q1 - q0)
    else:
        theta = np.arccos(dot)
        q = (np.sin((1 - t) * theta) * q0 + np.sin(t * theta) * q1) / np.sin(theta)
    return q / np.linalg.norm(q)


class PoseFilter:
    """Exponential smoothing of a pose that should be still (the keyboard, in a rover frame).
    alpha: weight of each new estimate, 1 = no smoothing. A jump bigger than reset_distance
    (m) or reset_angle (rad) restarts the filter (the keyboard or the rover moved)."""

    def __init__(self, alpha: float = 0.3, reset_distance: float = 0.05,
                 reset_angle: float = np.radians(10)):
        self.alpha = float(np.clip(alpha, 0.0, 1.0)) or 1.0
        self.reset_distance = reset_distance
        self.reset_angle = reset_angle
        self.position: np.ndarray | None = None
        self.quaternion: np.ndarray | None = None
        self.samples = 0

    def reset(self):
        self.position = self.quaternion = None
        self.samples = 0

    def update(self, position, quaternion) -> tuple[np.ndarray, np.ndarray]:
        p, q = np.asarray(position, float), np.asarray(quaternion, float)
        q = q / np.linalg.norm(q)
        if self.position is not None:
            angle = 2 * np.arccos(min(1.0, abs(float(np.dot(q, self.quaternion)))))
            if (np.linalg.norm(p - self.position) > self.reset_distance
                    or angle > self.reset_angle):
                self.reset()
        if self.position is None:
            self.position, self.quaternion = p, q
        else:
            self.position = (1 - self.alpha) * self.position + self.alpha * p
            self.quaternion = quaternion_slerp(self.quaternion, q, self.alpha)
        self.samples += 1
        return self.position, self.quaternion


def quaternion_to_rotation(q: Sequence[float]) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
