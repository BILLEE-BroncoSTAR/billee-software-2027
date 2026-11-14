"""ZED stereo over plain USB: camera discovery, factory calibration, CPU depth, point cloud.

Pure functions (no ROS), so they can be tested without a camera. Ported from Proj_1's
zed_usb/zed_usb_publisher.py.
"""
import configparser
import glob
import os
import urllib.request

import cv2
import numpy as np

CALIB_URL = 'https://calib.stereolabs.com/?SN={}'
# Per-eye image width -> calibration section suffix in SN<serial>.conf
RES_NAMES = {672: 'VGA', 1280: 'HD', 1920: 'FHD', 2208: '2K'}
# Side-by-side (both eyes) capture size to request per resolution name
REQUEST = {'VGA': (1344, 376), 'HD720': (2560, 720), 'HD1080': (3840, 1080), 'HD2K': (4416, 1242)}
STEREOLABS_USB_VENDOR = '2b03'


def read_sysfs(path):
    """Contents of a sysfs file, or '' if it is missing (e.g. the device was unplugged mid-scan)."""
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return ''


def find_zed(sys_root='/sys'):
    """Video device and serial number of the first ZED on USB (None for whichever is not found)."""
    # Numeric order: a ZED has a capture node and, right after it, a metadata node with the same
    # name; sorted as strings, video10 (metadata) would come before video9 (capture).
    nodes = sorted(glob.glob(f'{sys_root}/class/video4linux/video*'),
                   key=lambda p: int(os.path.basename(p)[len('video'):]))
    device = next((f'/dev/{os.path.basename(p)}' for p in nodes
                   if read_sysfs(f'{p}/name').startswith('ZED')), None)
    # The serial number is on the camera's HID (IMU) interface; the video interface reports a
    # placeholder.
    serial = next((read_sysfs(f'{d}/serial') for d in sorted(glob.glob(f'{sys_root}/bus/usb/devices/*'))
                   if read_sysfs(f'{d}/idVendor') == STEREOLABS_USB_VENDOR
                   and 'HID' in read_sysfs(f'{d}/product') and read_sysfs(f'{d}/serial')), None)
    return device, serial


def zed_sdk_present(root='/'):
    """True where the real ZED SDK can run (a Jetson, or a machine with the SDK installed)."""
    return (os.path.exists(os.path.join(root, 'etc/nv_tegra_release'))
            or os.path.exists(os.path.join(root, 'usr/local/zed/include/sl/Camera.hpp')))


def calibration_dirs(env=os.environ, cwd=None):
    """Where to look for SN<serial>.conf, first match wins.

    The repo's configs/zed/ (the team's cameras, committed so no internet is needed), the ZED
    SDK's own settings folder, then the per-user cache that downloads go to (the last entry).
    """
    cwd = cwd or os.getcwd()
    dirs = []
    for ws in (env.get('PIXI_PROJECT_ROOT'), env.get('BILLEE_WS'), cwd):
        if ws:
            dirs.append(os.path.normpath(os.path.join(ws, '..', 'configs', 'zed')))
    dirs += ['/usr/local/zed/settings', os.path.expanduser('~/.local/share/zed/settings')]
    seen = []
    for d in dirs:
        if d not in seen:
            seen.append(d)
    return seen


def find_calibration(serial, dirs):
    """Path of an existing SN<serial>.conf in dirs, else None."""
    for d in dirs:
        path = os.path.join(d, f'SN{serial}.conf')
        if os.path.isfile(path):
            return path
    return None


def download_calibration(serial, path):
    """Downloads SN<serial>.conf to `path`, which only appears once complete and valid: a partial
    download or an error page would otherwise be trusted on every later start."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    part = path + '.part'
    try:
        urllib.request.urlretrieve(CALIB_URL.format(serial), part)
        c = configparser.ConfigParser()
        try:
            c.read(part)
        except configparser.Error:
            pass
        if not c.has_section('STEREO'):
            raise RuntimeError(f'Calibration download for SN {serial} is not a ZED calibration file')
        os.replace(part, path)
    finally:
        if os.path.exists(part):
            os.remove(part)


def load_calibration(path, res):
    """OpenCV stereo calibration (Kl, Dl, Kr, Dr, R, T) from a ZED .conf, as Stereolabs'
    zed-open-capture builds it. `res` is a RES_NAMES value (VGA, HD, FHD, 2K)."""
    c = configparser.ConfigParser()
    c.read(path)

    def cam(side):
        s = c[f'{side}_CAM_{res}']
        K = np.array([[float(s['fx']), 0, float(s['cx'])], [0, float(s['fy']), float(s['cy'])], [0, 0, 1]])
        if f'{side}_DISTO' in c:
            # ZED 2/2i: OpenCV rational model; the k1-k3 in the resolution sections are only its
            # numerator. Using them alone gives a ~4x zoomed, wrong rectification.
            s = c[f'{side}_DISTO']
            D = np.array([float(s[k]) for k in ('k1', 'k2', 'p1', 'p2', 'k3', 'k4', 'k5', 'k6')])
        else:
            D = np.array([float(s[k]) for k in ('k1', 'k2', 'p1', 'p2', 'k3')])
        return K, D

    Kl, Dl = cam('LEFT')
    Kr, Dr = cam('RIGHT')
    st = c['STEREO']
    R = cv2.Rodrigues(np.array([float(st[f'RX_{res}']), float(st[f'CV_{res}']), float(st[f'RZ_{res}'])]))[0]
    T = np.array([float(st['Baseline']), float(st['TY']), float(st['TZ'])]) / 1000.0  # mm -> m
    return Kl, Dl, Kr, Dr, R, T


def flip_calibration(Kl, Dl, Kr, Dr, R, T, w, h):
    """Calibration for an upside-down camera: both images turned 180 degrees and left/right swapped."""
    F = np.diag([-1.0, -1.0, 1.0])

    def k(K):
        K = K.copy()
        K[0, 2], K[1, 2] = w - 1 - K[0, 2], h - 1 - K[1, 2]
        return K

    def d(D):
        D = D.copy()
        D[2:4] *= -1  # tangential p1, p2 change sign under a 180 degree turn; radial terms don't
        return D

    return k(Kr), d(Dr), k(Kl), d(Dl), F @ R.T @ F, -F @ R.T @ T


class Rectifier:
    """Rectification maps + SGBM matcher for one camera at one resolution."""

    def __init__(self, cal, w, h, min_depth, max_depth, flip=False):
        if flip:
            cal = flip_calibration(*cal, w, h)
        Kl, Dl, Kr, Dr, R, T = cal
        R1, R2, P1, P2, _, _, _ = cv2.stereoRectify(Kl, Dl, Kr, Dr, (w, h), R, T,
                                                    flags=cv2.CALIB_ZERO_DISPARITY, alpha=0)
        self.w, self.h, self.flip = w, h, flip
        self.map_l = cv2.initUndistortRectifyMap(Kl, Dl, R1, P1, (w, h), cv2.CV_16SC2)
        self.map_r = cv2.initUndistortRectifyMap(Kr, Dr, R2, P2, (w, h), cv2.CV_16SC2)
        self.P = P1
        self.baseline = abs(P2[0, 3] / P2[0, 0])
        self.fx_baseline = P1[0, 0] * self.baseline  # depth = fx * baseline / disparity
        self.min_depth, self.max_depth = min_depth, max_depth
        # Disparity search range covers everything from min_depth outwards.
        self.num_disp = int(np.ceil(self.fx_baseline / min_depth / 16.0)) * 16
        block = 5
        self.sgbm = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=self.num_disp, blockSize=block, P1=8 * block * block,
            P2=32 * block * block, disp12MaxDiff=1, uniquenessRatio=10, speckleWindowSize=100,
            speckleRange=2, mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)

    def split(self, frame):
        """Rectified (left, right) BGR images from one side-by-side frame."""
        left, right = frame[:, :self.w], frame[:, self.w:]
        if self.flip:
            left, right = cv2.rotate(right, cv2.ROTATE_180), cv2.rotate(left, cv2.ROTATE_180)
        return (cv2.remap(left, *self.map_l, cv2.INTER_LINEAR),
                cv2.remap(right, *self.map_r, cv2.INTER_LINEAR))

    def depth(self, left, right):
        """Depth in meters from a rectified pair; NaN where there is no match or it is out of range."""
        disp = self.sgbm.compute(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY),
                                 cv2.cvtColor(right, cv2.COLOR_BGR2GRAY)).astype(np.float32) / 16.0
        return disparity_to_depth(disp, self.fx_baseline, self.min_depth, self.max_depth)

    def alignment(self, frame):
        """(matches, median |dy| px, share with left x > right x) for a rectified textured frame,
        or None if the view has too little texture (fewer than 20 good ORB matches)."""
        left, right = self.split(frame)
        orb = cv2.ORB_create(1000)
        kl, dl = orb.detectAndCompute(cv2.cvtColor(left, cv2.COLOR_BGR2GRAY), None)
        kr, dr = orb.detectAndCompute(cv2.cvtColor(right, cv2.COLOR_BGR2GRAY), None)
        if dl is None or dr is None or len(dl) < 2 or len(dr) < 2:
            return None
        pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(dl, dr, k=2)
        m = [a for a, b in (p for p in pairs if len(p) == 2) if a.distance < 0.75 * b.distance]
        if len(m) < 20:
            return None
        dx = np.array([kl[x.queryIdx].pt[0] - kr[x.trainIdx].pt[0] for x in m])
        dy = np.array([kl[x.queryIdx].pt[1] - kr[x.trainIdx].pt[1] for x in m])
        good = np.abs(dy) < 3
        return len(m), float(np.median(np.abs(dy))), float((dx[good] > 0).mean()) if good.any() else 0.0


def disparity_to_depth(disp, fx_baseline, min_depth, max_depth):
    """Depth in meters from SGBM disparity in pixels; NaN where nothing matched or out of range."""
    with np.errstate(divide='ignore'):
        depth = np.where(disp > 0, fx_baseline / disp, np.nan).astype(np.float32)
    depth[(depth < min_depth) | (depth > max_depth)] = np.nan
    return depth


CLOUD_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<f4')])


def depth_to_cloud(depth, bgr, P, step=4):
    """Structured array (x, y, z, rgb) of the finite depth pixels, every `step`-th row and column.

    Points are in the left camera's ROS body frame (x forward, y left, z up), like zed_openpose's
    /zed/points (COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD). rgb is the colour packed as in
    PointCloud2 (0x00RRGGBB read as a float32).
    """
    step = max(1, int(step))
    fx, fy, cx, cy = P[0, 0], P[1, 1], P[0, 2], P[1, 2]
    z = depth[::step, ::step]
    v, u = np.mgrid[0:depth.shape[0]:step, 0:depth.shape[1]:step]
    keep = np.isfinite(z)
    z, u, v = z[keep], u[keep].astype(np.float32), v[keep].astype(np.float32)
    colour = bgr[::step, ::step][keep].astype(np.uint32)
    out = np.empty(z.size, dtype=CLOUD_DTYPE)
    out['x'] = z                       # optical z (forward)
    out['y'] = -(u - cx) * z / fx      # optical -x (left)
    out['z'] = -(v - cy) * z / fy      # optical -y (up)
    packed = (colour[:, 2] << 16) | (colour[:, 1] << 8) | colour[:, 0]
    out['rgb'] = packed.astype('<u4').view('<f4')
    return out
