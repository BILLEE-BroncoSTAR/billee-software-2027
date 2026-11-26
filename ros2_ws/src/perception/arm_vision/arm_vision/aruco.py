"""ArUco detection and 3D localisation for an RGB-D camera, without ROS (unit-tested on its own).

The approach follows zptang1210/RealsenseArUcoTracking (detect markers in the colour image, read
the aligned depth at each marker, deproject to a 3D point in the camera frame), reworked for the
rover:

  - OpenCV >= 4.7's ArucoDetector API (that repo uses Dictionary_get / DetectorParameters_create /
    drawMarker, which OpenCV 4.7 removed; the pixi environment has OpenCV 4.13).
  - Depth is the median over the marker's inner area instead of one centre pixel (that repo's
    TODO), so a hole or an edge pixel in the depth image doesn't throw the position off.
  - Intrinsics come from the ROS CameraInfo (K, D) instead of pyrealsense2, so the same code runs
    on a live camera, a rosbag or a simulated camera.
  - solvePnP (IPPE_SQUARE) gives each marker's orientation from its printed size, which the arm
    needs to line the gripper up, and a position when there is no valid depth.

Conventions: pixel (u, v) with v down; camera optical frame x right, y down, z forward; depth is
z (along the optical axis, as the RealSense reports it), in metres.
"""

from __future__ import annotations

import re
from collections.abc import Sequence
from dataclasses import dataclass, field

import cv2
import numpy as np

# Every predefined dictionary OpenCV has, by name. 5x5_50 by default: few ids -> large Hamming
# distance between codes, so few false ids, and 5x5 cells stay readable from further away than 6x6.
DICTIONARIES = {
    name: getattr(cv2.aruco, name)
    for name in (
        "DICT_4X4_50", "DICT_4X4_100", "DICT_4X4_250", "DICT_4X4_1000",
        "DICT_5X5_50", "DICT_5X5_100", "DICT_5X5_250", "DICT_5X5_1000",
        "DICT_6X6_50", "DICT_6X6_100", "DICT_6X6_250", "DICT_6X6_1000",
        "DICT_7X7_50", "DICT_7X7_100", "DICT_7X7_250", "DICT_7X7_1000",
        "DICT_ARUCO_ORIGINAL", "DICT_ARUCO_MIP_36h12",
        "DICT_APRILTAG_16h5", "DICT_APRILTAG_25h9", "DICT_APRILTAG_36h10", "DICT_APRILTAG_36h11",
    )
    if hasattr(cv2.aruco, name)
}

CORNER_REFINEMENT = {
    "none": cv2.aruco.CORNER_REFINE_NONE,
    "subpix": cv2.aruco.CORNER_REFINE_SUBPIX,
    "contour": cv2.aruco.CORNER_REFINE_CONTOUR,
    "apriltag": cv2.aruco.CORNER_REFINE_APRILTAG,
}

POSITION_DEPTH = "depth"
POSITION_PNP = "pnp"
POSITION_SOURCES = ("auto", POSITION_DEPTH, POSITION_PNP)


def get_dictionary(name: str):
    try:
        return cv2.aruco.getPredefinedDictionary(DICTIONARIES[name])
    except KeyError:
        raise ValueError(
            f"unknown ArUco dictionary {name!r}; one of: {', '.join(DICTIONARIES)}") from None


@dataclass
class Intrinsics:
    """Pinhole camera matrix K (3x3) and distortion D, as in sensor_msgs/CameraInfo."""

    K: np.ndarray
    D: np.ndarray = field(default_factory=lambda: np.zeros(5))

    @classmethod
    def from_camera_info(cls, k: Sequence[float], d: Sequence[float]) -> Intrinsics:
        K = np.asarray(k, dtype=np.float64).reshape(3, 3)
        D = np.asarray(d, dtype=np.float64).ravel()
        # OpenCV takes 4, 5, 8, 12 or 14 coefficients; RealSense colour streams report 5
        # (plumb_bob / inverse Brown-Conrady, all close to 0). Anything else: no distortion.
        if D.size not in (4, 5, 8, 12, 14):
            D = np.zeros(5)
        return cls(K=K, D=D)

    @property
    def valid(self) -> bool:
        return bool(self.K[0, 0] > 0 and self.K[1, 1] > 0)


@dataclass
class Marker:
    """One detected marker. position / quaternion are in the camera optical frame."""

    id: int
    corners: np.ndarray                   # (4, 2) px: TL, TR, BR, BL (OpenCV order)
    center: np.ndarray                    # (2,) px: where the diagonals cross
    depth: float = float("nan")           # median aligned depth over the marker (m)
    position: np.ndarray | None = None    # (3,) m
    position_source: str = ""
    quaternion: np.ndarray | None = None  # (x, y, z, w), marker frame -> camera frame
    rvec: np.ndarray | None = None
    tvec: np.ndarray | None = None
    reprojection_error: float = float("nan")

    @property
    def distance(self) -> float:
        return float(np.linalg.norm(self.position)) if self.position is not None else float("nan")


class ArucoDetector:
    """Detects markers in a BGR or grey image and locates them in 3D.

    marker_size: printed side length of the black square (m), outer edge to outer edge, not
    counting the white margin. 0 = unknown: no orientation and no PnP position.
    marker_sizes: {id: size} for markers printed at another size (e.g. the keyboard's).
    """

    def __init__(
        self,
        dictionary: str = "DICT_5X5_50",
        marker_size: float = 0.05,
        corner_refinement: str = "subpix",
        position_source: str = "auto",
        min_depth: float = 0.1,
        max_depth: float = 6.0,
        depth_inner_fraction: float = 0.6,
        min_valid_depth_fraction: float = 0.25,
        marker_sizes: dict[int, float] | None = None,
        full_frame_every: int = 1,
        track_margin: float = 0.75,
    ):
        """full_frame_every > 1: tracking. Between full-frame scans (every full_frame_every
        frames) only padded boxes around the last frame's markers are searched, at full
        resolution, so a frame costs a few ms instead of 11-25 ms on the Jetson. A marker that
        isn't found again triggers a full scan at once; a new one shows up at the next full scan.
        track_margin: box padding, as a fraction of the marker's size (min 24 px)."""
        if position_source not in POSITION_SOURCES:
            raise ValueError(f"position_source {position_source!r}, expected {POSITION_SOURCES}")
        if corner_refinement not in CORNER_REFINEMENT:
            raise ValueError(f"corner_refinement {corner_refinement!r}, expected "
                             f"{tuple(CORNER_REFINEMENT)}")
        self.dictionary_name = dictionary
        self.dictionary = get_dictionary(dictionary)
        self.marker_size = float(marker_size)
        self.marker_sizes = {int(k): float(v) for k, v in (marker_sizes or {}).items()}
        self.position_source = position_source
        self.min_depth = float(min_depth)
        self.max_depth = float(max_depth)
        self.depth_inner_fraction = float(depth_inner_fraction)
        self.min_valid_depth_fraction = float(min_valid_depth_fraction)

        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = CORNER_REFINEMENT[corner_refinement]
        self._detector = cv2.aruco.ArucoDetector(self.dictionary, params)
        self.full_frame_every = max(1, int(full_frame_every))
        self.track_margin = float(track_margin)
        self._frames = 0
        self._tracked: list[Marker] = []   # last frame's markers
        self.last_full_scan = True         # whether the last detect() scanned the whole frame

    def detect(self, image: np.ndarray) -> list[Marker]:
        """2D detection only: ids, corners and centres, sorted by id."""
        gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self._frames += 1
        markers = None
        if (self.full_frame_every > 1 and self._tracked
                and self._frames % self.full_frame_every != 0):
            markers = self._detect_tracked(gray)
        self.last_full_scan = markers is None
        if markers is None:
            markers = self._detect_in(gray, 0, 0)
        markers.sort(key=lambda m: m.id)
        self._tracked = markers
        return markers

    def _detect_in(self, gray: np.ndarray, x0: int, y0: int) -> list[Marker]:
        corners, ids, _rejected = self._detector.detectMarkers(gray)
        if ids is None:
            return []
        out = []
        for c, marker_id in zip(corners, ids.ravel()):
            c = np.asarray(c, dtype=np.float64).reshape(4, 2) + (x0, y0)
            out.append(Marker(id=int(marker_id), corners=c, center=marker_center(c)))
        return out

    def _detect_tracked(self, gray: np.ndarray) -> list[Marker] | None:
        """Search boxes around the tracked markers; None = do a full scan instead (a tracked
        marker wasn't found again, or the boxes cover most of the frame anyway)."""
        h, w = gray.shape
        boxes = []
        for m in self._tracked:
            lo, hi = m.corners.min(axis=0), m.corners.max(axis=0)
            pad = max(24.0, self.track_margin * float(max(hi - lo)))
            x0, y0 = (int(max(0, v - pad)) for v in lo)
            x1, y1 = int(min(w, hi[0] + pad)), int(min(h, hi[1] + pad))
            boxes.append([x0, y0, x1, y1])
        merged = []  # overlapping boxes would find the same marker twice: merge them
        for b in sorted(boxes):
            for m in merged:
                if b[0] < m[2] and b[2] > m[0] and b[1] < m[3] and b[3] > m[1]:
                    m[:] = [min(m[0], b[0]), min(m[1], b[1]), max(m[2], b[2]), max(m[3], b[3])]
                    break
            else:
                merged.append(b)
        if sum((x1 - x0) * (y1 - y0) for x0, y0, x1, y1 in merged) > 0.5 * w * h:
            return None
        found: dict[int, Marker] = {}
        for x0, y0, x1, y1 in merged:
            for m in self._detect_in(gray[y0:y1, x0:x1], x0, y0):
                found.setdefault(m.id, m)
        if {m.id for m in self._tracked} - set(found):
            return None  # lost one (moved fast, or out of its box): look everywhere
        return list(found.values())

    def locate(
        self,
        markers: list[Marker],
        intrinsics: Intrinsics | None,
        depth: np.ndarray | None = None,
    ) -> list[Marker]:
        """Fills in depth, position and orientation of each marker, in place (and returns it).

        depth: aligned to the colour image (same size and pixel grid), metres, 0 / NaN = none.
        Without intrinsics only `depth` is filled in.
        """
        for m in markers:
            if depth is not None:
                m.depth = marker_depth(depth, m.corners, self.depth_inner_fraction,
                                       self.min_depth, self.max_depth,
                                       self.min_valid_depth_fraction)
            if intrinsics is None or not intrinsics.valid:
                continue
            size = self.size_of(m.id)
            if size > 0:
                pose = estimate_pose(m.corners, size, intrinsics)
                if pose is not None:
                    m.rvec, m.tvec, m.reprojection_error = pose
                    m.quaternion = rotation_to_quaternion(cv2.Rodrigues(m.rvec)[0])
            use_depth = self.position_source in ("auto", POSITION_DEPTH) and np.isfinite(m.depth)
            if use_depth:
                m.position = deproject(m.center, m.depth, intrinsics)
                m.position_source = POSITION_DEPTH
            elif self.position_source in ("auto", POSITION_PNP) and m.tvec is not None:
                m.position = m.tvec.ravel().copy()
                m.position_source = POSITION_PNP
        return markers

    def size_of(self, marker_id: int) -> float:
        return self.marker_sizes.get(marker_id, self.marker_size)

    def process(self, image, intrinsics=None, depth=None) -> list[Marker]:
        return self.locate(self.detect(image), intrinsics, depth)


def parse_marker_sizes(items) -> dict[int, float]:
    """["10:0.03", "11:0.03"] -> {10: 0.03, 11: 0.03} (ROS parameters can't be maps).

    An entry may name a range of ids, "10-13:0.03", and `items` may also be one string with the
    entries separated by commas or spaces (a launch argument: "10-13:0.03,7:0.1")."""
    if isinstance(items, str):
        items = re.split(r"[\s,]+", items)
    sizes = {}
    for item in items:
        if not item.strip():
            continue
        ids, sep, size = item.strip().partition(":")
        try:
            first, _, last = ids.partition("-")
            first, last = int(first), int(last or first)
            size = float(size)
        except ValueError:
            first = None
        if not sep or first is None or not 0 <= first <= last or not size > 0:
            raise ValueError(f"marker_sizes entry {item!r}: expected 'id:size_m' or "
                             "'first-last:size_m', size in metres > 0")
        for marker_id in range(first, last + 1):
            sizes[marker_id] = size
    return sizes


def format_marker_sizes(sizes: dict[int, float]) -> list[str]:
    """{10: 0.03} -> ["10:0.03"], the marker_sizes parameter's format."""
    return [f"{marker_id}:{size:g}" for marker_id, size in sorted(sizes.items())]


def marker_center(corners: np.ndarray) -> np.ndarray:
    """Where the marker's diagonals cross: its true centre under perspective (the corner mean
    is off-centre for a tilted marker). Falls back to the mean for a degenerate quad."""
    p0, p1, p2, p3 = corners
    d1, d2 = p2 - p0, p3 - p1
    denom = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(denom) < 1e-9:
        return corners.mean(axis=0)
    t = ((p1[0] - p0[0]) * d2[1] - (p1[1] - p0[1]) * d2[0]) / denom
    return p0 + t * d1


def marker_depth(
    depth: np.ndarray,
    corners: np.ndarray,
    inner_fraction: float = 0.6,
    min_depth: float = 0.1,
    max_depth: float = 6.0,
    min_valid_fraction: float = 0.25,
) -> float:
    """Median valid depth (m) inside the marker, shrunk about its centre by inner_fraction so the
    background just past the edges doesn't count. NaN if too few pixels have valid depth."""
    center = marker_center(corners)
    inner = center + (corners - center) * inner_fraction
    h, w = depth.shape[:2]
    x0, y0 = np.floor(inner.min(axis=0)).astype(int)
    x1, y1 = np.ceil(inner.max(axis=0)).astype(int) + 1
    x0, y0, x1, y1 = max(x0, 0), max(y0, 0), min(x1, w), min(y1, h)
    if x1 <= x0 or y1 <= y0:
        return float("nan")
    mask = np.zeros((y1 - y0, x1 - x0), dtype=np.uint8)
    cv2.fillConvexPoly(mask, np.round(inner - [x0, y0]).astype(np.int32), 1)
    if not mask.any():  # marker a few px wide: use the centre pixel
        mask[min(max(int(center[1]) - y0, 0), mask.shape[0] - 1),
             min(max(int(center[0]) - x0, 0), mask.shape[1] - 1)] = 1
    values = depth[y0:y1, x0:x1][mask.astype(bool)].astype(np.float64)
    valid = values[np.isfinite(values) & (values >= min_depth) & (values <= max_depth)]
    if valid.size == 0 or valid.size < min_valid_fraction * values.size:
        return float("nan")
    return float(np.median(valid))


def deproject(pixel: Sequence[float], z: float, intrinsics: Intrinsics) -> np.ndarray:
    """Pixel + depth (z, m) -> 3D point in the camera optical frame, undistorting the pixel."""
    pts = np.asarray(pixel, dtype=np.float64).reshape(1, 1, 2)
    x, y = cv2.undistortPoints(pts, intrinsics.K, intrinsics.D).reshape(2)
    return np.array([x * z, y * z, z])


def marker_object_points(size: float) -> np.ndarray:
    """The marker's corners in its own frame, in OpenCV's detection order (what IPPE_SQUARE
    expects): x right, y up, z out of the face, origin at the centre."""
    h = size / 2.0
    return np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])


def estimate_pose(
    corners: np.ndarray, size: float, intrinsics: Intrinsics
) -> tuple[np.ndarray, np.ndarray, float] | None:
    """solvePnP for one square marker -> (rvec, tvec, RMS reprojection error px), or None."""
    obj = marker_object_points(size)
    img = np.asarray(corners, dtype=np.float64).reshape(4, 1, 2)
    try:
        ok, rvec, tvec = cv2.solvePnP(obj, img, intrinsics.K, intrinsics.D,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
    except cv2.error:
        return None
    if not ok or tvec[2, 0] <= 0:
        return None
    projected, _ = cv2.projectPoints(obj, rvec, tvec, intrinsics.K, intrinsics.D)
    error = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - img.reshape(4, 2)) ** 2,
                                         axis=1))))
    return rvec, tvec, error


def rotation_to_quaternion(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> unit quaternion (x, y, z, w), w >= 0."""
    m = np.asarray(R, dtype=np.float64)
    trace = np.trace(m)
    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, s / 4]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [s / 4, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 1] + m[1, 0]) / s, s / 4, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, s / 4, (m[1, 0] - m[0, 1]) / s]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def draw_markers(
    image: np.ndarray,
    markers: list[Marker],
    intrinsics: Intrinsics | None = None,
    marker_size: float = 0.0,
    marker_sizes: dict[int, float] | None = None,
) -> np.ndarray:
    """Annotated copy of a BGR image: outline, id and distance of each marker, plus its axes
    (x red, y green, z blue) where it has a pose."""
    out = image.copy() if image.ndim == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if not markers:
        return out
    corners = [m.corners.reshape(1, 4, 2).astype(np.float32) for m in markers]
    cv2.aruco.drawDetectedMarkers(out, corners, None, borderColor=(0, 255, 0))
    for m in markers:
        size = (marker_sizes or {}).get(m.id, marker_size)
        if m.rvec is not None and intrinsics is not None and intrinsics.valid and size > 0:
            cv2.drawFrameAxes(out, intrinsics.K, intrinsics.D, m.rvec, m.tvec, size * 0.75, 2)
        cx, cy = round(m.center[0]), round(m.center[1])
        cv2.circle(out, (cx, cy), 4, (0, 0, 255), -1)
        label = f"id {m.id}"
        if m.position is not None:
            label += f"  {m.distance:.2f} m ({m.position_source})"
        org = (int(m.corners[:, 0].min()), int(m.corners[:, 1].max()) + 18)
        cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
                    cv2.LINE_AA)
    return out


def generate_marker(dictionary: str, marker_id: int, side_px: int = 600,
                    margin_px: int | None = None) -> np.ndarray:
    """Printable marker image: the marker side_px wide plus a white quiet zone around it (one
    cell wide by default; detection needs it). Print so the black square is marker_size wide."""
    d = get_dictionary(dictionary)
    if not 0 <= marker_id < len(d.bytesList):
        raise ValueError(f"{dictionary} has ids 0-{len(d.bytesList) - 1}, not {marker_id}")
    img = cv2.aruco.generateImageMarker(d, marker_id, side_px, borderBits=1)
    if margin_px is None:
        margin_px = side_px // (d.markerSize + 2)
    return cv2.copyMakeBorder(img, margin_px, margin_px, margin_px, margin_px,
                              cv2.BORDER_CONSTANT, value=255)
