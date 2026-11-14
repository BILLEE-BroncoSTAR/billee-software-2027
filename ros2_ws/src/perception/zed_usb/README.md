# zed_usb

The ZED 2i on a machine **without an NVIDIA GPU** (a laptop ground station, a dev box). The ZED SDK
needs CUDA, so `zed_openpose_node` only builds in the Jetson image. This node reads the camera's
side-by-side USB video instead, rectifies it with the factory calibration, computes depth on the
CPU (OpenCV SGBM) and publishes what the rest of the stack reads from the ZED.

```bash
tooling/zed-usb                       # from the repo root: camera + rover model TF + Foxglove :8765
tooling/zed-usb flip:=true            # camera mounted upside down
ros2 launch zed_usb zed_usb.launch.py standalone:=false   # next to a drivetrain launch
ros2 launch chassis_bringup rover.launch.py camera:=cloud # uses zed_usb where zed_openpose isn't built
```

| Topic | Type | Notes |
|---|---|---|
| `/zed/points` | `PointCloud2` (x y z rgb) | `zed_left_camera_frame`, every 4th pixel, ≤ 10 Hz: same as `zed_openpose_node` |
| `/zed/zed_node/rgb/color/rect/image`, `…/camera_info` | `Image` bgr8 | rectified left, `zed_left_camera_frame_optical` (ZED ROS 2 wrapper names) |
| `/zed/zed_node/depth/depth_registered`, `…/camera_info` | `Image` 32FC1 | metres, NaN = no depth |
| TF (static) | | `camera_link` → `zed_left_camera_frame` (half a baseline, +y) → `…_optical` |

Image and depth messages are only built while something subscribes.

**Calibration:** `SN<serial>.conf`, first found in `configs/zed/` (committed, no internet needed),
`/usr/local/zed/settings/`, `~/.local/share/zed/settings/`; otherwise downloaded once from
Stereolabs into the last one.

**Differences from the ZED SDK:** classic stereo matching, not neural depth, so blank walls have
holes; no IMU, no OpenPose (it needs CUDA). About 60 ms of CPU per VGA frame on a laptop, so
`resolution:=VGA` (the default) is the practical setting; a USB 2 link only carries VGA anyway.

**Refuses to start** where the ZED SDK can run (a Jetson, or `/usr/local/zed` installed; use
`camera:=cloud` there, or `force:=true`), and when another node already publishes `/zed/points`.

Troubleshooting:

- *Waiting for a ZED camera on USB*: `lsusb | grep 2b03`; in Docker the container needs `/dev`
  (the x86 devcontainer and `tooling/desktop-ros2` mount it).
- *Asked for HD720, the camera gives VGA*: the cable or port is USB 2. Flip the USB-C plug or use a
  USB 3 port.
- On USB 3 the camera re-enumerates every few seconds while idle (autosuspend); it is stable while
  streaming, and the node reconnects by itself.
- *Rectification check … looks wrong*: the camera is upside down (`flip:=true`) or the
  calibration belongs to another serial. The check needs a textured scene; it skips blank views.

Ported from Proj_1's `zed_usb/zed_usb_publisher.py`. Tests: `test/test_stereo.py` (team calibration
+ synthetic frames, no camera needed).
