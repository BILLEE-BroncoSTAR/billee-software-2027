# Arm camera: RealSense D435i + ArUco

An Intel RealSense D435i on top of the arm's end effector gives colour, depth aligned to the colour
image, and an IMU. The rover runs the driver and a CV pipeline on the Jetson (l4t container): the
first CV stage finds ArUco markers and works out each one's 3D position and orientation. The ground
station watches the annotated feed in Foxglove.

Packages: `perception/arm_vision` (launch, configs, the CV nodes) and `perception/arm_vision_msgs`
(`ArucoMarker`, `ArucoMarkers`). The ArUco approach follows
[zptang1210/RealsenseArUcoTracking](https://github.com/zptang1210/RealsenseArUcoTracking); see
[Relation to RealsenseArUcoTracking](#relation-to-realsensearucotracking).

```
D435i ──USB 3──> realsense2_camera (/arm_cam) ──> color/image_raw, aligned_depth_to_color/image_raw,
 (librealsense, libusb backend)                   color/camera_info, imu, TF arm_cam_link -> *_optical_frame
                                                        │
                                                        ▼
                                  arm_vision aruco_node (/arm_cam/aruco_node), via rgbd.py
                                  ├─ aruco/markers            arm_vision_msgs/ArucoMarkers
                                  ├─ aruco/image/compressed   annotated JPEG  ──> ground station (Foxglove)
                                  ├─ aruco/visualization      MarkerArray
                                  └─ TF  arm_cam_color_optical_frame -> aruco_<id>
```

- [Setup (Jetson)](#setup-jetson)
- [Run](#run)
- [Topics and frames](#topics-and-frames)
- [ArUco markers](#aruco-markers)
- [Adding CV stages](#adding-cv-stages)
- [Mounting on the arm](#mounting-on-the-arm)
- [Parameters](#parameters)
- [How it was built](#how-it-was-built)
- [Status](#status)
- [Troubleshooting](#troubleshooting)

## Setup (Jetson)

Everything comes with the l4t dev container. There's nothing to install by hand and no kernel patch.

- **SDK and driver** come from pixi (`ros2_ws/pixi.toml`): `ros-humble-realsense2-camera` 4.57.7 and
  conda-forge's `librealsense` 2.57.7, built with the **RSUSB (libusb) backend**. That backend is
  Intel's recommended setup on a Jetson: it needs no patched `uvcvideo` and no DKMS modules, and
  the IMU works over it too. The same package provides `pyrealsense2` and the `rs-*` tools
  (`rs-enumerate-devices`, `rs-fw-update`).
- **USB permissions:** libusb needs read/write on the camera's `/dev/bus/usb` node. The image ships
  Intel's `99-realsense-libusb.rules` (`docker/`), and every container start installs it on the host
  (`docker/realsense_udev.sh`, from `postStartCommand`), as `zed_udev.sh` does for the ZED.

After pulling this branch on the Jetson:

1. **Rebuild the container**: **Dev Containers: Rebuild Container**. Only the small udev layers
   change, so OpenPose isn't recompiled. `postStartCommand` then runs `pixi install`, which
   downloads the RealSense packages.
2. Plug the D435i into a **USB 3** port (blue, or marked SS) with a USB 3 cable. On the **host**:
   ```bash
   lsusb | grep 8086          # Intel Corp. ... RealSense (8086:0b3a = D435i)
   lsusb -t                   # its line must say 5000M, not 480M (USB 2)
   ls -l /etc/udev/rules.d/99-realsense-libusb.rules
   ```
3. **Jetson container**, build:
   ```bash
   cd ros2_ws && pixi shell -e l4t
   pixi run build --packages-up-to arm_vision chassis_bringup
   source install/local_setup.bash
   rs-enumerate-devices -s    # serial, firmware, USB type 3.2: the SDK sees the camera
   ```
4. Recommended once: update the camera firmware to the version librealsense 2.57 recommends
   (`rs-enumerate-devices` prints both the current and the recommended versions). Download the
   `.bin` from Intel's [D400 firmware page](https://dev.intelrealsense.com/docs/firmware-releases-d400),
   then run `rs-fw-update -f <file>.bin`. Don't unplug the camera during the update.

## Run

**Jetson, with the rover:** `rover.launch.py` starts the arm camera by itself whenever a RealSense is
plugged in (`arm_camera:=auto`, the default):

```bash
ros2 launch chassis_bringup rover.launch.py mode:=real                    # + arm camera if plugged in
ros2 launch chassis_bringup rover.launch.py mode:=real arm_camera:=true   # start it and wait for it
ros2 launch chassis_bringup rover.launch.py mode:=real arm_camera:=false  # never
```

**Jetson, on its own** (camera + ArUco + Foxglove bridge on :8765):

```bash
ros2 launch arm_vision arm_camera.launch.py
ros2 launch arm_vision arm_camera.launch.py marker_size:=0.1 dictionary:=DICT_4X4_50
ros2 launch arm_vision arm_camera.launch.py aruco:=false      # camera only
ros2 launch arm_vision arm_camera.launch.py camera:=false     # ArUco only, e.g. on a rosbag of /arm_cam/*
ros2 launch arm_vision arm_camera.launch.py serial_no:=838212070123   # a specific camera
```

**Ground station:** Foxglove Studio → `ws://<rover>:8765` (or `ws://localhost:8765` when the ground
station runs its own bridge, as with `ground_station.launch.py`).
- `rover_dashboard.json` (chassis_bringup), the Foxglove layout, has an **Arm RealSense** panel
  showing the annotated feed. It reads "waiting for messages" until the camera is up.
- In RViz, add an Image display on `/arm_cam/aruco/image` with transport `compressed`.

Bandwidth: the bridge sends only what a panel subscribes to. The annotated JPEG is about 10-40 kB
at 15 Hz (well under 1 Mbit/s). `/arm_cam/color/image_raw/compressed` is JPEG at 30 Hz. **Don't
subscribe to the raw images or `aligned_depth_to_color/image_raw` from the ground station**: that's
27 MB/s of colour over Wi-Fi.

## Topics and frames

| Topic | Type | |
|---|---|---|
| `/arm_cam/color/image_raw` (+ `/compressed`) | `sensor_msgs/Image` | colour, 640×480 @ 30, rgb8 |
| `/arm_cam/color/camera_info` | `sensor_msgs/CameraInfo` | colour intrinsics (factory calibration) |
| `/arm_cam/aligned_depth_to_color/image_raw` (+ `/compressedDepth`) | `sensor_msgs/Image` | depth on the colour pixels, 16UC1 mm |
| `/arm_cam/depth/image_rect_raw` | `sensor_msgs/Image` | depth, 848×480 |
| `/arm_cam/imu` | `sensor_msgs/Imu` | gyro + accel, linear interpolation (`unite_imu_method: 2`) |
| `/arm_cam/aruco/markers` | `arm_vision_msgs/ArucoMarkers` | every processed frame; empty list = no marker |
| `/arm_cam/aruco/image/compressed` | `sensor_msgs/CompressedImage` | annotated JPEG (only encoded while subscribed) |
| `/arm_cam/aruco/visualization` | `visualization_msgs/MarkerArray` | plate + label per marker |

TF (the driver publishes these as static transforms): `arm_cam_link` → `arm_cam_color_frame` →
`arm_cam_color_optical_frame` (and the depth, infra and IMU frames). The ArUco node adds
`arm_cam_color_optical_frame` → `aruco_<id>` for each marker it locates. Frame names start with
`arm_cam_` (the driver's `camera_name`), so they can't collide with the ZED's `camera_link`.

`ArucoMarker` (see `arm_vision_msgs/msg/ArucoMarker.msg`): `id`, `pose` (optical frame), `position_source`
(`depth` | `pnp`), `has_orientation`, `distance`, `depth`, `reprojection_error` and the four
`corners` in pixels.

## ArUco markers

**Print markers** at a known size:

```bash
ros2 run arm_vision aruco_generate --ids 0 1 2 3 --size-mm 50            # DICT_5X5_50, ./markers/*.png
ros2 run arm_vision aruco_generate --dictionary DICT_4X4_50 --ids 7 --size-mm 100 --dpi 600
```

Print at 100 % ("actual size", not "fit to page"), measure the black square, and set
`marker_size` (metres) to that. Keep the white border: detection needs it. Mount the marker flat.

**How a marker is located** (`arm_vision/aruco.py`):

1. `cv2.aruco.ArucoDetector` finds the markers' ids and corners in the colour image, with sub-pixel
   corner refinement.
2. **Orientation:** `solvePnP` (`SOLVEPNP_IPPE_SQUARE`) from the four corners and `marker_size`. The
   marker frame follows OpenCV's convention: x along the top edge, y up the marker, z out of the
   printed face toward the camera.
3. **Position:** the median aligned depth over the inner 60 % of the marker (holes and flying pixels
   are ignored), deprojected at the marker's centre (where its diagonals cross) through the colour
   intrinsics, lens distortion included. If fewer than 25 % of those pixels have valid depth (too
   close, under about 0.2-0.3 m for the D435i, too far, or a glossy marker), it uses the PnP
   translation instead. `position_source` says which was used; `position_source:=depth|pnp` forces
   one.

Depth is accurate whatever `marker_size` says. PnP is only as good as `marker_size` and corner
accuracy: a 1 mm error on a 50 mm marker is 2 % of the distance. The synthetic tests recover
position within 5 mm at 0.6 m (depth) and orientation within 3°.

**Choosing a dictionary:** `DICT_5X5_50` is the default. With only 50 ids the codes are far apart,
so false ids are rare, and 5×5 cells stay readable further away than 6×6. Use `DICT_4X4_50` for
small markers or long range. Set `marker_ids` to ignore the ids you don't use.

## Adding CV stages

`arm_vision/rgbd.py` is the input layer for any CV node on the arm camera. `RgbdSubscriber` pairs
each colour image with the aligned depth image carrying the same stamp, waiting at most one frame
for it, and never with an older one (which would misplace things while the arm moves). It hands
over an `RgbdFrame` with `color` (BGR), `depth` (metres, or `None`), `intrinsics` (K, D) and
`header`:

```python
from arm_vision.rgbd import RgbdFrame, RgbdSubscriber

class GraspPointNode(Node):
    def __init__(self):
        super().__init__("grasp_point_node")
        self.rgbd = RgbdSubscriber(self, self.on_frame)   # relative topics: run it in /arm_cam

    def on_frame(self, frame: RgbdFrame):
        ...  # frame.color, frame.depth, frame.intrinsics; arm_vision.aruco.deproject() for 3D
```

Add it to `setup.py`'s `console_scripts` and to `arm_camera.launch.py` (namespace `arm_cam`).
Call `cv2.setNumThreads(1)` as `aruco_node` does: OpenCV's thread pool contending with the other
cameras for the Jetson's cores made ArUco detection about 10× slower.

## Mounting on the arm

The camera's TF tree starts at `arm_cam_link`. Until the arm has a URDF, nothing attaches it to the
rover. For now, `arm_camera.launch.py` can publish the mount as a static transform:

```bash
ros2 launch arm_vision arm_camera.launch.py parent_frame:=end_effector mount_z:=0.04 mount_pitch:=0.0
```

When the arm URDF exists (see `arm_description` on the `B-004-Base-Arm-Sim` branch, and
`arm_bringup` on `kevin/arm-bringup`), mount the camera there with
`realsense2_description`'s macro (already in pixi), and drop `parent_frame:=`:

```xml
<xacro:include filename="$(find realsense2_description)/urdf/_d435i.urdf.xacro"/>
<xacro:sensor_d435i parent="end_effector" name="arm_cam" use_nominal_extrinsics="false">
  <origin xyz="0 0 0.04" rpy="0 0 0"/>
</xacro:sensor_d435i>
```

With `name="arm_cam"`, the macro's root link is `arm_cam_link` and its mesh matches the real
camera. Set `publish_tf: false` in `config/realsense_d435i.yaml` if the URDF then publishes the
camera's internal frames too (only one publisher per frame). Markers then reach every frame of the
arm through TF, for example `ros2 run tf2_ros tf2_echo base_link aruco_3`.

## Parameters

`config/realsense_d435i.yaml` (driver) and `config/aruco.yaml` (ArUco node) are commented. The
settings most worth knowing:

| | Default | |
|---|---|---|
| `rgb_camera.color_profile` | `640,480,30` | `1280,720,30` sees markers about twice as far, for about twice the CPU |
| `depth_module.depth_profile` | `848,480,30` | the D435i's best depth resolution |
| `align_depth.enable` | `true` | needed for depth positions |
| `enable_gyro` / `enable_accel` | `true` | IMU on `/arm_cam/imu` |
| `initial_reset` | `false` | `true` if the camera opens but streams nothing after a Jetson reboot |
| `marker_size` | `0.05` | m, black square only |
| `dictionary` | `DICT_5X5_50` | |
| `position_source` | `auto` | `depth` when valid, else `pnp` |
| `depth_sync_slop` | `0.02` | s, colour/depth stamp match |
| `opencv_threads` | `1` | |
| `debug_image_rate` / `_quality` / `_width` | `15` / `75` / `640` | the ground station feed |

## How it was built

What the l4t container does for the RealSense, and why:

- **librealsense from conda-forge, not built in the Dockerfile.** ROS here comes from pixi
  (RoboStack, Python 3.12). An SDK built against the system Python or system ROS wouldn't load in
  that environment, and RoboStack's `realsense2_camera` is already built against conda-forge's
  librealsense. conda-forge builds it with `FORCE_RSUSB_BACKEND=ON` and mentions Jetson boards
  explicitly.
- **The CPU build** (`build = "cpu_py312*"` in `pixi.toml`). Its CUDA builds need CUDA ≥ 12.9, while
  JetPack 6 has 12.6. CUDA only speeds up colour conversion and alignment, which 640×480 doesn't
  need. `py312`: each build bundles `pyrealsense2` for one Python version without declaring which.
- **Version 4.57.7**, the newest `realsense2_camera` built for this lock's RoboStack build
  (`ros2-distro-mutex 0.9`, Python 3.12). `pixi.toml` pins that mutex. Without the pin, adding any
  package lets the solver move all of ROS to the next RoboStack build (Python 3.14).
- **glibc 2.34** is declared on both pixi platforms (`glibc = "2.34"`; every container is Ubuntu
  22.04 with glibc 2.35). RoboStack's librealsense2 for build 18 needs `libudev1 >= 260`, which needs
  it. pixi only allows that on a named platform entry, hence `linux-aarch64-glibc234`.

## Relation to RealsenseArUcoTracking

[zptang1210/RealsenseArUcoTracking](https://github.com/zptang1210/RealsenseArUcoTracking) is a
standalone Windows script: pyrealsense2 grabs aligned frames, `cv2.aruco` finds markers, the centre
pixel's depth is deprojected with `rs2_deproject_pixel_to_point`, and points are plotted with
matplotlib. That repository has no licence file, so no code was copied. `arm_vision` reimplements
the same approach as ROS 2 nodes and changes it as follows:

| RealsenseArUcoTracking | arm_vision |
|---|---|
| `cv2.aruco.Dictionary_get`, `DetectorParameters_create`, `drawMarker` (removed in OpenCV 4.7) | `ArucoDetector`, `generateImageMarker` (OpenCV 4.13 in pixi) |
| depth of the one centre pixel (its TODO: "use fillPoly to mask the marker") | median over the marker's inner area, holes and outliers rejected |
| centre = midpoint of two corners | centre = where the diagonals cross (correct under perspective) |
| position only | + orientation (solvePnP), + PnP fallback without depth |
| intrinsics from pyrealsense2 | from `camera_info`: works on a live camera, a rosbag or a sim |
| `cv2.imshow` window | ROS topics, TF, annotated JPEG for the ground station |
| trajectory kept in a dict, plotted every 20 s | `/arm_cam/aruco/markers`: record it with `ros2 bag`, plot it in Foxglove |
| `ArUcoGenerator.py` | `aruco_generate`, sized for printing (DPI in the PNG) |

## Status

Tested in a cloud container, x86 with no camera attached:
- `pytest` (`arm_vision/test`, 31 tests): detection, depth and PnP positions and orientation against
  ground truth on synthetic RGB-D frames, depth holes, lens distortion, colour/depth pairing, USB
  detection, and marker generation.
- End to end over ROS: a synthetic D435i stream on the driver's topics → `aruco_node` (launched by
  `arm_camera.launch.py camera:=false`) → markers, TF, MarkerArray and annotated JPEG, with the
  position within 1 mm of the ground truth.
- `arm_camera.launch.py` with the real driver: `realsense2_camera` 4.57.7 / librealsense 2.57.7 starts,
  reports "No RealSense devices were found", and keeps waiting; the mount TF and the bridge come up;
  Ctrl-C stops everything cleanly. `rover.launch.py arm_camera:=auto|true|false` picks the right one.

**Not yet tested: a real D435i on the Jetson.** First run checklist: `rs-enumerate-devices -s`
shows USB 3.2; `ros2 topic hz /arm_cam/color/image_raw` ≈ 30; `aruco_node` logs about 30 fps with
"100% with depth"; a printed marker at a measured distance reads within a centimetre; the IMU
publishes on `/arm_cam/imu`.

Tests:
```bash
cd ros2_ws/src/perception/arm_vision   # inside pixi shell
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q test   # (pytest 9 vs the launch_testing plugin, as in people-pose.md)
```

## Troubleshooting

- **`No RealSense devices were found!`** (repeats every 6 s):
  - On the host, `lsusb | grep 8086`. If nothing shows, it's the cable or port.
  - `ls -l /dev/bus/usb/*/*` should show the camera's node as `crw-rw-rw-`. If not, the udev rule
    is missing: rerun `sudo /usr/local/bin/realsense_udev.sh`, then unplug and replug.
  - If `rs-enumerate-devices` works as root but not as `ros_user`, it's the same udev rule.
- **USB 2** (`arm_camera.launch.py` warns "USB 2 link"; `rs-enumerate-devices` says `2.1`): the
  D435i then can't stream colour and depth at 30 fps. Use a USB 3 port and cable, and plug the
  USB-C end in fully (half-inserted USB-C often negotiates USB 2). Extension cables and passive
  hubs are a common cause.
- **Frames stop after a while / `Frames didn't arrive within 5 seconds`:** cable strain on the
  moving arm, or USB autosuspend. Disable autosuspend on the host:
  `echo -1 | sudo tee /sys/module/usbcore/parameters/autosuspend`. The driver reconnects by itself
  (`reconnect_timeout`).
- **`recovery mode`** (the launch warns, PID `0adb`): a firmware update failed. Run `rs-fw-update -r -f <fw>.bin`.
- **The camera opens but no images arrive after a reboot:** set `initial_reset: true`.
- **Markers seen but `position_source: pnp`, `depth: NaN`:** the marker is closer than the D435i's
  minimum depth (about 0.2-0.3 m), too far away, or glossy. Check `align_depth.enable` and that
  `/arm_cam/aligned_depth_to_color/image_raw` is publishing. Or `use_depth` is false.
- **Wrong distances with `pnp`:** `marker_size` doesn't match the printed square.
- **`aruco_node` logs low fps / high ms per frame:** check `opencv_threads` (1), and the colour
  resolution.
- **`parameter 'serial_no' has invalid type`:** pass it through `arm_camera.launch.py serial_no:=`,
  which quotes it; a bare numeric serial in a YAML file becomes a number.
