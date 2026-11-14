#!/usr/bin/env python3
"""zed_usb_node: a ZED 2i over plain USB, for machines without an NVIDIA GPU.

The ZED SDK needs CUDA, so zed_openpose_node only exists in the Jetson image. This node reads the
camera's side-by-side UVC video instead, rectifies it with the factory calibration, computes depth
on the CPU (OpenCV SGBM) and publishes what the rest of the stack reads from the ZED:

  /zed/points                                  PointCloud2  x y z rgb, zed_left_camera_frame
                                               (same topic, frame and layout as zed_openpose)
  /zed/zed_node/rgb/color/rect/image           Image        bgr8, rectified left
  /zed/zed_node/rgb/color/rect/camera_info     CameraInfo
  /zed/zed_node/depth/depth_registered         Image        32FC1 metres, NaN = no depth
  /zed/zed_node/depth/camera_info              CameraInfo   (the ZED ROS 2 wrapper's topic names)
  TF (static) camera_link -> zed_left_camera_frame -> zed_left_camera_frame_optical

Depth is classic stereo matching, not the SDK's neural depth: expect holes on blank walls.
No OpenPose (it needs CUDA). Ported from Proj_1's zed_usb/zed_usb_publisher.py.

It refuses to start where the real ZED SDK can run (Jetson, or /usr/local/zed installed) unless
force:=true, and when another node already publishes /zed/points.
"""
import array
import configparser
import os
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

from zed_usb import stereo

WRAPPER_ROOT = '/zed/zed_node/'  # the ZED ROS 2 wrapper's topics (camera 'zed', node 'zed_node')
CLOUD_TOPIC = '/zed/points'      # zed_openpose_node's point cloud


class Refused(Exception):
    """This machine or ROS graph shouldn't run zed_usb (message says why)."""


CLOUD_FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate(('x', 'y', 'z', 'rgb'))]


class ZedUsb(Node):
    def __init__(self):
        super().__init__('zed_usb')
        self.declare_parameter('device', '')  # '' = first ZED found
        # CPU depth takes ~60 ms/frame at VGA on a laptop; HD720 has 4x the pixels.
        self.declare_parameter('resolution', 'VGA')  # VGA, HD720, HD1080, HD2K (USB 2 only carries VGA)
        # Capture faster than depth runs: at 15 fps every other frame was missed (7.5 Hz out).
        self.declare_parameter('fps', 30)
        self.declare_parameter('flip', False)  # true if the camera is mounted upside down
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('calib_dir', '')  # '' = repo configs/zed, ZED SDK settings, ~/.local cache
        self.declare_parameter('point_cloud', True)
        self.declare_parameter('cloud_max_hz', 10.0)  # cap on /zed/points; 0 = every frame
        self.declare_parameter('cloud_step', 4)       # keep every Nth pixel (zed_openpose's default)
        self.declare_parameter('camera_frame', 'camera_link')  # URDF frame of the ZED's centre
        self.declare_parameter('zed_frame', 'zed_left_camera_frame')
        self.declare_parameter('force', False)  # run even where the ZED SDK could
        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.preflight(p('force'))
        self.flip, self.point_cloud = p('flip'), p('point_cloud')
        self.cloud_period = 1.0 / p('cloud_max_hz') if p('cloud_max_hz') > 0 else 0.0
        self.cloud_step = p('cloud_step')
        self.zed_frame = p('zed_frame')
        self.optical_frame = self.zed_frame + '_optical'
        self.failures = 0  # consecutive failed reads
        self.cap = None
        self.last_cloud = None
        self.depth_filled = 0.0

        self.fixed_device = p('device')
        resolution = p('resolution')
        if resolution not in stereo.REQUEST:
            self.get_logger().warn(f'Unknown resolution {resolution!r}, using VGA '
                                   f'(choices: {", ".join(stereo.REQUEST)})')
            resolution = 'VGA'
        rw, rh = stereo.REQUEST[resolution]
        self.request = (rw, rh, p('fps'))
        # Never exits on a missing or misbehaving camera: waits and retries until a usable frame.
        device, serial, frame = self.wait_for_camera()
        w, h = frame.shape[1] // 2, frame.shape[0]
        res = stereo.RES_NAMES[w]
        if (w * 2, h) != (rw, rh):
            self.get_logger().warn(f'Asked for {resolution}, the camera gives {res} {w}x{h} (a USB 2 '
                                   'link only carries VGA: use a USB 3 port / flip the USB-C plug)')
        dirs = [p('calib_dir')] if p('calib_dir') else stereo.calibration_dirs()
        cal = self.wait_for_calibration(serial, res, dirs)
        self.rect = stereo.Rectifier(cal, w, h, p('min_depth'), p('max_depth'), self.flip)

        self.pub_rgb = self.create_publisher(Image, WRAPPER_ROOT + 'rgb/color/rect/image', 10)
        self.pub_rgb_info = self.create_publisher(CameraInfo, WRAPPER_ROOT + 'rgb/color/rect/camera_info', 10)
        self.pub_depth = self.create_publisher(Image, WRAPPER_ROOT + 'depth/depth_registered', 10)
        self.pub_depth_info = self.create_publisher(CameraInfo, WRAPPER_ROOT + 'depth/camera_info', 10)
        self.pub_cloud = self.create_publisher(PointCloud2, CLOUD_TOPIC, 1)
        self.publish_static_tf(p('camera_frame'))
        self.get_logger().info(
            f'ZED SN {serial} on {device}: {res} {w}x{h}, baseline {self.rect.baseline:.4f} m, '
            f'fx {self.rect.P[0, 0]:.1f}, flip {self.flip}, depth {self.rect.min_depth}-'
            f'{self.rect.max_depth} m ({self.rect.num_disp} disparities), CPU stereo (no ZED SDK)')
        self.check_alignment(frame)

    def preflight(self, force):
        """Raises Refused where this node shouldn't run."""
        if stereo.zed_sdk_present() and not force:
            raise Refused(
                'The ZED SDK can run here (Jetson, or /usr/local/zed installed): use its depth '
                '(rover.launch.py camera:=cloud, or zed_openpose.launch.py) instead of CPU stereo. '
                'force:=true runs zed_usb anyway.')
        # Never mix CPU stereo with the SDK's point cloud: give discovery a moment, then check.
        time.sleep(2.0)
        others = other_cloud_sources(self)
        if others:
            raise Refused(f'{", ".join(others)} already publishes {CLOUD_TOPIC}; not starting a second '
                          'ZED source')

    # ----------------------------------------------------------------------------- camera
    def wait_for_camera(self, retry=1.0):
        """(device, serial, first frame) once a ZED is found and gives a frame of a known size."""
        warned = False
        while True:
            device, serial = stereo.find_zed()
            device = self.fixed_device or device
            if device and serial:
                if warned:
                    time.sleep(2.0)  # let the camera finish starting up after it appears
                try:
                    self.cap = self.open(device)
                    ok, frame = self.cap.read()
                except cv2.error as e:
                    ok, frame = False, None
                    self.get_logger().warn(f'Cannot open {device}: {e}')
                if ok and frame is not None and frame.ndim == 3 and frame.shape[1] // 2 in stereo.RES_NAMES:
                    return device, serial, frame
                if self.cap is not None:
                    self.cap.release()
                shape = 'no frame' if not ok or frame is None else f'unexpected frame size {frame.shape}'
                self.get_logger().warn(f'ZED on {device} is not delivering video yet ({shape}), retrying',
                                       throttle_duration_sec=10.0)
            elif not warned:
                self.get_logger().warn('Waiting for a ZED camera on USB (lsusb | grep 2b03)...')
            warned = True
            time.sleep(retry)

    def wait_for_calibration(self, serial, res, dirs, retry=5.0):
        """Stereo calibration, downloading the factory file first if no folder has it; retries if
        offline or invalid."""
        while True:
            calib = stereo.find_calibration(serial, dirs)
            try:
                if calib is None:
                    calib = os.path.join(dirs[-1], f'SN{serial}.conf')
                    self.get_logger().info(f'Downloading factory calibration for SN {serial} to {calib}')
                    stereo.download_calibration(serial, calib)
                else:
                    self.get_logger().info(f'Calibration: {calib}')
                return stereo.load_calibration(calib, res)
            except (OSError, RuntimeError, KeyError, ValueError, configparser.Error) as e:
                self.get_logger().error(
                    f'No usable calibration for SN {serial} ({type(e).__name__}: {e}); retrying in '
                    f'{retry:.0f} s. Check the network, or put SN{serial}.conf in configs/zed/ '
                    '(https://calib.stereolabs.com/?SN=<serial>)')
                if calib and not isinstance(e, OSError) and calib.startswith(dirs[-1]) and os.path.exists(calib):
                    os.replace(calib, calib + '.bad')  # corrupt download: fetch it again next time
                time.sleep(retry)

    def open(self, device):
        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        rw, rh, fps = self.request
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, rw)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, rh)
        cap.set(cv2.CAP_PROP_FPS, fps)
        return cap

    def reopen(self):
        """After a USB reset the ZED comes back under a new /dev/video number: find it and reopen."""
        self.cap.release()
        device = self.fixed_device or stereo.find_zed()[0]
        if device:
            try:
                self.cap = self.open(device)
                ok, frame = self.cap.read()
            except cv2.error:
                ok, frame = False, None
            if ok and frame is not None and frame.shape[1] // 2 == self.rect.w and frame.shape[0] == self.rect.h:
                self.get_logger().info(f'Camera back on {device}')
                return True
            self.cap.release()
        time.sleep(1.0)
        return False

    def check_alignment(self, frame):
        """Warns if rectification looks wrong: matches should share rows, with left x > right x."""
        result = self.rect.alignment(frame)
        if result is None:
            self.get_logger().info('Rectification check skipped: too little texture in view')
            return
        n, dy, share = result
        msg = f'Rectification check: {n} matches, median |dy| {dy:.2f} px, {share * 100:.0f}% with left x > right x'
        if dy > 1.5 or share < 0.8:
            self.get_logger().warn(msg + ' -> calibration or flip looks wrong')
        else:
            self.get_logger().info(msg)

    # ----------------------------------------------------------------------------- output
    def publish_static_tf(self, camera_frame):
        # The left lens is half a baseline to the left (+y) of the camera's centre, as in
        # zed_openpose_node; the optical frame is the usual z-forward, x-right, y-down rotation.
        stamp = self.get_clock().now().to_msg()
        lens = TransformStamped()
        lens.header.stamp, lens.header.frame_id, lens.child_frame_id = stamp, camera_frame, self.zed_frame
        lens.transform.translation.y = self.rect.baseline / 2.0
        lens.transform.rotation.w = 1.0
        optical = TransformStamped()
        optical.header.stamp, optical.header.frame_id = stamp, self.zed_frame
        optical.child_frame_id = self.optical_frame
        r = optical.transform.rotation
        r.x, r.y, r.z, r.w = -0.5, 0.5, -0.5, 0.5
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform([lens, optical])

    def info(self, stamp):
        m = CameraInfo()
        m.header.stamp, m.header.frame_id = stamp, self.optical_frame
        m.width, m.height, m.distortion_model = self.rect.w, self.rect.h, 'plumb_bob'
        m.d = [0.0] * 5  # images are already rectified
        m.k = self.rect.P[:, :3].flatten().tolist()
        m.r = np.eye(3).flatten().tolist()
        m.p = self.rect.P.flatten().tolist()
        return m

    def image(self, stamp, img, encoding):
        m = Image()
        m.header.stamp, m.header.frame_id = stamp, self.optical_frame
        m.height, m.width = img.shape[:2]
        m.encoding = encoding
        m.step = img.strides[0]
        # array.array skips the per-byte check rclpy runs on bytes (~100 ms per frame at VGA).
        m.data = array.array('B', img.tobytes())
        return m

    def cloud(self, stamp, depth, bgr):
        pts = stereo.depth_to_cloud(depth, bgr, self.rect.P, self.cloud_step)
        m = PointCloud2()
        m.header.stamp, m.header.frame_id = stamp, self.zed_frame
        m.height, m.width = 1, pts.size
        m.fields = CLOUD_FIELDS
        m.is_bigendian, m.point_step, m.is_dense = False, stereo.CLOUD_DTYPE.itemsize, True
        m.row_step = m.point_step * m.width
        m.data = array.array('B', pts.tobytes())
        return m

    def step(self):
        """Reads, processes and publishes one frame. Returns False if the camera gave nothing usable.
        Never raises because of the camera: a failed read reconnects, a bad frame is skipped."""
        try:
            ok, frame = self.cap.read()
        except cv2.error:
            ok, frame = False, None
        if not ok or frame is None:
            self.failures += 1
            if self.failures == 1:
                self.get_logger().warn('Camera read failed (USB disconnect?), reconnecting')
            self.reopen()
            return False
        self.failures = 0
        if frame.ndim != 3 or frame.shape[:2] != (self.rect.h, self.rect.w * 2):
            self.get_logger().warn(f'Skipping frame of unexpected size {frame.shape}', throttle_duration_sec=5.0)
            return False
        now = self.get_clock().now()
        stamp = now.to_msg()
        try:
            left, right = self.rect.split(frame)
            depth = self.rect.depth(left, right)
        except cv2.error as e:
            self.get_logger().warn(f'Skipping frame: {e}', throttle_duration_sec=5.0)
            return False
        # Build a message only for topics someone listens to: each costs a few ms at VGA.
        if self.pub_rgb.get_subscription_count():
            self.pub_rgb.publish(self.image(stamp, left, 'bgr8'))
        if self.pub_rgb_info.get_subscription_count():
            self.pub_rgb_info.publish(self.info(stamp))
        if self.pub_depth.get_subscription_count():
            self.pub_depth.publish(self.image(stamp, depth, '32FC1'))
        if self.pub_depth_info.get_subscription_count():
            self.pub_depth_info.publish(self.info(stamp))
        due = self.last_cloud is None or (now - self.last_cloud).nanoseconds * 1e-9 >= self.cloud_period
        if self.point_cloud and due and self.pub_cloud.get_subscription_count():
            self.pub_cloud.publish(self.cloud(stamp, depth, left))
            self.last_cloud = now
        self.depth_filled = float(np.isfinite(depth).mean())
        return True

    def spin(self):
        frames, since = 0, time.monotonic()
        while rclpy.ok():
            frames += self.step()
            rclpy.spin_once(self, timeout_sec=0)
            if time.monotonic() - since > 10.0:  # a health line every 10 s
                self.get_logger().info(f'{frames / (time.monotonic() - since):.1f} Hz, depth '
                                       f'{self.depth_filled * 100:.0f}% filled')
                frames, since = 0, time.monotonic()


def other_cloud_sources(node):
    """Nodes other than this one publishing /zed/points (e.g. zed_openpose_node), as /ns/name."""
    me = f'{node.get_namespace().rstrip("/")}/{node.get_name()}'
    infos = node.get_publishers_info_by_topic(CLOUD_TOPIC)
    names = [f'{i.node_namespace.rstrip("/")}/{i.node_name}' for i in infos]
    return [n for n in names if n != me]


def main():
    rclpy.init()
    node = None
    code = 0
    try:
        node = ZedUsb()
        node.spin()
    except Refused as e:
        rclpy.logging.get_logger('zed_usb').error(str(e))
        code = 1
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if node.cap is not None:
                node.cap.release()
            node.destroy_node()
        rclpy.try_shutdown()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
