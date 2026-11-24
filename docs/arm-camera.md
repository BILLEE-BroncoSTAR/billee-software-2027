# Arm camera: RealSense D435i, ArUco, keyboard

An Intel RealSense D435i on top of the arm's end effector gives colour, depth aligned to the colour
image, and an IMU. The rover runs the driver and a CV pipeline on the Jetson (l4t container): the
first CV stage finds ArUco markers and works out each one's 3D position and orientation. The second
locates a keyboard from the markers stuck on it, and gives every key's position relative to the
rover, so the arm can type. The arm's URDF (`arm_description`) carries the camera on the end
effector, so TF places the camera, the markers and the keyboard relative to `base_link`. The ground
station watches the annotated feed in Foxglove.

Packages: `perception/arm_vision` (launch, configs, the CV nodes) and `perception/arm_vision_msgs`
(`ArucoMarker`, `ArucoMarkers`, `Keyboard`, `KeyboardKey`), and `arm_description` (the arm on the rover,
with the camera). The ArUco approach follows
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
                                                        │ aruco/markers
                                                        ▼
                                  arm_vision keyboard_node (/arm_cam/keyboard_node)
       TF base_link <- arm_cam_color_optical_frame      ├─ keyboard/keys    arm_vision_msgs/Keyboard
       (arm_description + /arm/joint_states) ──────────>├─ keyboard/pose    PoseStamped (base_link)
                                                        ├─ keyboard/visualization  MarkerArray
                                                        └─ TF  base_link -> keyboard
```

- [Setup (Jetson)](#setup-jetson)
- [Run](#run)
- [Topics and frames](#topics-and-frames)
- [ArUco markers](#aruco-markers)
- [Keyboard](#keyboard)
- [Arm URDF: the camera on the rover](#arm-urdf-the-camera-on-the-rover)
- [Adding CV stages](#adding-cv-stages)
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
ros2 launch arm_vision arm_camera.launch.py marker_sizes:=10-13:0.04   # keyboard markers: 40 mm
ros2 launch arm_vision arm_camera.launch.py aruco:=false      # camera only
ros2 launch arm_vision arm_camera.launch.py camera:=false     # ArUco only, e.g. on a rosbag of /arm_cam/*
ros2 launch arm_vision arm_camera.launch.py serial_no:=838212070123   # a specific camera
ros2 launch arm_vision arm_camera.launch.py arm_tf:=zero      # keyboard in base_link, arm not running
ros2 launch arm_vision arm_camera.launch.py target_frame:=    # keyboard in the camera frame only
```

**From the BILLEE dashboard:** **Teleop · Arm** runs all of this. Set **Drivetrain** to Real or Sim
(Gazebo on the Jetson) and **Arm** to Real, Mock (no hardware: jogging moves the arm model and the
camera) or Off (joints held at 0, e.g. a RealSense on a testbed Jetson). See the dashboard's README,
"Teleop · Arm: real or sim". In `mode:=sim` the arm camera and the arm's TF stay on the wall clock,
since the RealSense is real hardware. **ArUco marker sizes (mm)** (Advanced) sets the printed
sizes without editing the config files: `40` = every marker, `10-13:30` = those ids, or both
(`rover.launch.py marker_size:=` / `marker_sizes:=`, which `rover.launch.py` hands to
`arm_camera.launch.py`).

**Marker sizes at launch:** `marker_size:=` (m) replaces `config/aruco.yaml`'s default size.
`marker_sizes:=` takes `id:size_m` or `first-last:size_m` entries separated by commas; they are
merged over `aruco.yaml`'s `marker_sizes` (ids not named keep theirs) for `aruco_node`, and replace
the layout's size of those markers for `keyboard_node`, so both locate the keyboard's markers with
the same size. A malformed entry stops the launch with the entry in the error.

The arm's TF comes up with `rover.launch.py` (`arm_tf:=auto`) or `arm_camera.launch.py`
(`arm_tf:=auto`), and moves with `/arm/joint_states` from `arm_bringup`'s `arm.launch.py`, which is
started separately. See [Arm URDF](#arm-urdf-the-camera-on-the-rover).

**Ground station:** Foxglove Studio → `ws://<rover>:8765` (or `ws://localhost:8765` when the ground
station runs its own bridge, as with `ground_station.launch.py`).
- `rover_dashboard.json` (chassis_bringup), the Foxglove layout, has an **Arm RealSense** panel
  showing the annotated feed with the located keyboard's keys drawn over it, and a **3D: arm**
  panel with the arm model, TF, the markers and the keyboard around `base_link`. They read
  "waiting for messages" until the camera is up.
- RViz: `chassis_bringup/rviz/drivetrain.rviz` has the arm model, the marker and keyboard
  MarkerArrays and the annotated feed; `ros2 launch arm_description arm_description.launch.py
  rviz:=true` opens a view of just the arm (`arm_description/rviz/arm.rviz`).

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
| `/arm_cam/keyboard/keys` | `arm_vision_msgs/Keyboard` | keyboard pose + every key, in `target_frame` (`base_link`) |
| `/arm_cam/keyboard/pose` | `geometry_msgs/PoseStamped` | the keyboard frame in `target_frame` |
| `/arm_cam/keyboard/visualization` | `visualization_msgs/MarkerArray` | outline, keycaps, labels, the layout's markers (keyboard frame) |
| `/arm/robot_description` | `std_msgs/String` | the arm's URDF (arm_description) |

TF (the driver publishes these as static transforms): `arm_cam_link` → `arm_cam_color_frame` →
`arm_cam_color_optical_frame` (and the depth, infra and IMU frames). The ArUco node adds
`arm_cam_color_optical_frame` → `aruco_<id>` for each marker it locates. Frame names start with
`arm_cam_` (the driver's `camera_name`), so they can't collide with the ZED's `camera_link`.

Up to the camera, the arm's robot_state_publisher (`/arm`, arm_description) publishes
`base_link` → `arm_base_link` → (`joint_arm_1` … `joint_arm_5`) → `arm_link_5_endeff_1` →
`arm_tool_link` → `arm_camera_mount` → `arm_cam_bottom_screw_frame` → `arm_cam_link`. The keyboard
node adds `base_link` → `keyboard`.

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

## Keyboard

`keyboard_node` (`arm_vision/keyboard_node.py`, the maths in `arm_vision/keyboard.py`) finds the
keyboard from ArUco markers stuck on it, and publishes the keyboard's pose and every key's position
relative to the rover. It's in `arm_camera.launch.py` (`keyboard:=true`, the default) and so in
`rover.launch.py`.

**Set up a keyboard** (`config/keyboard_layout.yaml`):

1. Print four markers, ids 10-13 (`DICT_5X5_50`), with a 30 mm black square:
   `ros2 run arm_vision aruco_generate --ids 10 11 12 13 --size-mm 30`, or
   [chev.me/arucogen](https://chev.me/arucogen/) (dictionary 5x5, size 30 mm). Print at 100 %.
2. Stick them flat on the keyboard case, or on a plate the keyboard is fixed to, upright to the
   typist, near the four corners. They must not move relative to the keys.
3. Measure each marker's centre in the **keyboard frame** and write it in `markers:`:
   origin at the front-left corner of the key field (outer corner of the left Ctrl), x to the right
   along the rows, y away from the typist, z up; z = 0 is the key tops, so a marker on the case
   has a negative z. A rotated marker gets `yaw:` (degrees), a different size `size:`.
4. Check `key_unit` (distance between two neighbouring letter keys, 19.05 mm on most keyboards)
   and `rows:` (ANSI US by default, F-row included; arrows and the numpad are left out).
5. If the markers' size isn't 30 mm, change `marker_sizes` in `config/aruco.yaml` too, or set it
   at launch for both nodes: `marker_sizes:=10-13:0.04` (dashboard: **ArUco marker sizes**
   `10-13:40`).

**How it's located:** every `aruco/markers` message that has layout markers in it is one estimate.

1. `solvePnP` over the corners of every layout marker in view, as one rigid object. Its starting
   pose comes from the largest marker alone (`IPPE_SQUARE`). With several markers, the spacing
   in the layout sets the scale, so a marker printed slightly off-size barely matters.
2. The translation is shifted so the markers' predicted centres meet their measured aligned-depth
   positions on average (`use_depth`). This corrects PnP's weak axis, the distance, unless the two
   disagree by more than `max_depth_correction`.
3. Fits with an RMS reprojection error over `max_reprojection_error` are rejected (a marker moved,
   or the layout is mismeasured), and the node's 10 s status line says why.
4. TF to `target_frame` (`base_link`) at the image's stamp, through the arm's joint angles and the
   camera mount. Then smoothing in that frame (`smoothing`), since the keyboard should be still
   there. A jump of more than `reset_distance` / `reset_angle` restarts the filter.
5. The last estimate is held and republished (`tracking: false`) while no marker is in view, so
   the key positions survive the arm getting too close to see the markers. `hold_timeout` drops
   it after a while (0 = never); `ros2 service call /arm_cam/keyboard_node/reset
   std_srvs/srv/Trigger` drops it now (do that after the rover drives).

On synthetic frames (`test/test_keyboard.py`, D435i-like intrinsics, camera 0.45 m above the
keyboard and tilted): with all four markers, every key is within 0.2 mm and 0.03° of the truth;
with two markers, within 2 mm; with one marker, within 8 mm. The real error is dominated by how
well the markers' positions are measured and by the arm's kinematics (next section).

**Using it:** `arm_vision_msgs/Keyboard` on `/arm_cam/keyboard/keys` lists every key's top centre
in `base_link`. A key is pressed along the keyboard's -z. `arm_vision.keyboard.keys_for_text()`
turns text into key presses for the layout, for example `"Hi!"` becomes `[("H", "Shift"), ("I",),
("1", "Shift")]`:

```python
from arm_vision.keyboard import KeyboardLayout, keys_for_text
layout = KeyboardLayout.from_yaml(".../config/keyboard_layout.yaml")
for key, *modifiers in keys_for_text("Hi!", layout):
    ...  # look the labels up in the latest Keyboard message, hold modifiers, press key
```

## Arm URDF: the camera on the rover

`arm_description` is the arm on the rover, with the D435i on its end effector:

- `urdf/arm_model.urdf.xacro`: the arm's links and joints. **Placeholder**: the B-004 arm model
  (`B-004-Base-Arm-Sim` branch), until the new arm's URDF is ready. It's generated by
  `import_urdf.py` (below), not hand-edited.
- `urdf/arm.urdf.xacro`: the macro `billee_arm(parent, prefix, camera_xyz, camera_rpy)`. It fixes
  the model's root to the rover, adds `arm_tool_link` on the model's tip link (the end effector),
  then `arm_camera_mount` and `realsense2_description`'s `sensor_d435i` (`name="arm_cam"`,
  `use_nominal_extrinsics="false"`: the macro adds only `arm_cam_link` and the mesh, and the
  driver publishes the calibrated optical frames, so there is one publisher per frame).
- `urdf/arm_standalone.urdf.xacro`: the robot the arm's `robot_state_publisher` loads. Its root
  is an empty `base_link`, so it joins the rover's TF tree without changing `robot_description`,
  which the Gazebo sim is built from.
- `launch/arm_description.launch.py`: `robot_state_publisher` in `/arm`, on
  `/arm/joint_states`.

**Joint angles:** the URDF's movable joints are named after `arm_bringup` (`config/arm_axes.yaml`:
`joint_arm_1` … `joint_arm_5`), so `arm.launch.py`'s `joint_state_broadcaster`
(`/arm/joint_states`) drives TF directly. The fingers are fixed, because nothing publishes their
state. `joint_states:=zero` holds every joint at 0 (bench tests without the arm; never together with
`arm.launch.py`, since that makes two publishers), and `joint_states:=gui` gives sliders to check
the model and the mount by eye. Without joint states, TF stops at `arm_base_link`, and
`keyboard_node` warns that there is no TF from the camera to `base_link`.

**Placeholders to measure** (launch arguments of `arm_description.launch.py`, xacro args):

| | Default | |
|---|---|---|
| `mount_xyz` / `mount_rpy` | `0.25 0 0.329` / `0 0 0` | arm base in `base_link` (0.329 m is the top of the chassis mesh) |
| `camera_xyz` / `camera_rpy` | `-0.09 0 0.045` / `0 0 π` | D435i bottom screw in `arm_tool_link`: on top of the end effector, looking along the fingers |

**Accuracy:** a key's position relative to the rover is only as good as this chain: the URDF's
link lengths, the joint zero offsets and directions (`arm_axes.yaml`: the RoboClaw axes 4-5 read
0 rad wherever they are at power-on, so they need homing first), the camera mount, and then the
keyboard estimate. For typing, re-detect the keyboard from close to the keys, and check one key
with the arm before typing a string.

### Importing the new arm (Kevin)

When the new arm's URDF exists, re-run the importer; nothing else needs editing:

```bash
cd ros2_ws/src/arm_description
python3 arm_description/import_urdf.py path/to/new_arm/urdf/new_arm.urdf \
    --tip-link <end effector link> \
    --joint-map <exported joint 1>:joint_arm_1 <exported joint 2>:joint_arm_2 ... <joint 5>:joint_arm_5 \
    --meshes path/to/new_arm/meshes
# (or `ros2 run arm_description import_urdf ...` once built). The placeholder was made with:
#   import_urdf.py arm_model.xacro --tip-link link_5_endeff_1 \
#     --joint-map joint_axis_1:joint_arm_1 ... joint_axis_5:joint_arm_5
```

It takes a `.urdf` or an exporter's `.xacro`, prefixes the links with `arm_` (the rover already has
a `base_link`), renames the joints to `arm_bringup`'s names, fixes the movable joints you leave
out, points the meshes at `package://arm_description/meshes/`, and drops `<ros2_control>`,
`<gazebo>` and `<transmission>` (`arm_bringup` generates its own `ros2_control` from
`arm_axes.yaml`). Then:

1. Delete the old meshes from `meshes/` that the new model doesn't use.
2. Set `mount_*` and `camera_*` from the real arm, and each axis's `direction` in `arm_axes.yaml`
   so positive radians turn the way the URDF's axis says.
3. Check it: `ros2 launch arm_description arm_description.launch.py joint_states:=gui rviz:=true`;
   the tests (`python -m pytest -q test` in `arm_description`) check that the movable joints are
   `arm_axes.yaml`'s and that every mesh is there.

With no URDF at all, `arm_camera.launch.py parent_frame:=<link> mount_x:=...` publishes a fixed
mount for `arm_cam_link` instead (and doesn't start the arm's TF).

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

## Parameters

`config/realsense_d435i.yaml` (driver), `config/aruco.yaml` (ArUco node) and `config/keyboard.yaml`
(keyboard node) are commented. The settings most worth knowing:

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
| `marker_sizes` | `["10:0.03", ...]` | per-id sizes (the keyboard's markers); `"10-13:0.03"` ranges too. Launch: `marker_sizes:=` sets aruco_node's and keyboard_node's |
| keyboard `target_frame` | `base_link` | `""` = camera frame |
| keyboard `min_markers` / `max_reprojection_error` | `1` / `4.0` | px RMS |
| keyboard `smoothing` / `hold_timeout` | `0.3` / `0` | `0` = hold until `~/reset` |

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
- `pytest` (`arm_vision/test`, 47 tests): detection, depth and PnP positions and orientation against
  ground truth on synthetic RGB-D frames, depth holes, lens distortion, colour/depth pairing, USB
  detection, marker generation, per-id marker sizes; and the keyboard: layout parsing, text → keys,
  the pose from synthetic frames of the four markers (all, two, one; depth on and off; a marker
  printed 3 % small; a marker moved 4 cm, which is rejected), and the smoothing filter.
- `pytest` (`arm_description/test`, 6 tests): `import_urdf.py`, and that the model's movable joints
  are `arm_axes.yaml`'s and every mesh is present. `xacro arm_standalone.urdf.xacro` expands
  (against realsense2_description 4.57.7's macros) into a valid 14-link tree from `base_link` to
  `arm_cam_link`.
- End to end over ROS: a synthetic D435i stream on the driver's topics → `aruco_node` (launched by
  `arm_camera.launch.py camera:=false`) → markers, TF, MarkerArray and annotated JPEG, with the
  position within 1 mm of the ground truth.
- `arm_camera.launch.py` with the real driver: `realsense2_camera` 4.57.7 / librealsense 2.57.7 starts,
  reports "No RealSense devices were found", and keeps waiting; the mount TF and the bridge come up;
  Ctrl-C stops everything cleanly. `rover.launch.py arm_camera:=auto|true|false` picks the right one.

On the testbed Jetson (Orin Nano, JetPack 6.2.1, l4t container), 2026-11-23, with a real D435i
(firmware 5.15.1, USB 3.2) and no arm: `xvfb-run -a vglrun -d egl ros2 launch chassis_bringup
rover.launch.py mode:=sim arm_tf:=zero` (what the dashboard runs for Teleop · Arm: Sim, Arm Off):
- The driver, `aruco_node`, `keyboard_node`, the arm's `robot_state_publisher` (`/arm`, joints held
  at 0) all start, and Ctrl-C stops them cleanly. TF `base_link` ← `arm_cam_color_optical_frame`
  resolves through the arm.
- `/arm_cam/imu` 200 Hz. Colour about 24 Hz by `ros2 topic hz`, `aruco_node` 21-24 fps at
  28-36 ms/frame, 80-93 % with depth: the CPU is saturated in sim (Gazebo takes a core). Plain
  `cv2.aruco` detection measured 21-24 ms per 640×480 frame there, single-threaded.
- `keyboard_node` under ROS, fed synthetic RGB-D frames of markers 10-13 (this D435i's intrinsics,
  stamped in its optical frame, so through the live arm TF to `base_link`): all four markers
  0.5 mm / 0.05° from the truth at 0.41 px RMS, markers 10 + 13 only 1.4 mm / 0.8°. The estimate is
  held (`tracking: false`) with no markers in view, and `~/reset` drops it.
- Fixed on that run: `keyboard_node` crashed on its first MarkerArray (an int in a float field,
  which the stubbed messages it was written against allowed), and its TF listener's spin thread
  also spun the whole node in a second executor. The launch log printed the camera's USB serial,
  which on a D400 is not the `serial_no` the driver takes.

**Not yet tested: printed markers in front of the real camera.** Remaining checklist: a printed
marker at a measured distance reads within a centimetre; with markers 10-13 in view,
`/arm_cam/aruco/markers` lists them, `keyboard_node` logs a reprojection error under ~1-2 px, and
a key's position (`/arm_cam/keyboard/keys`, `target_frame:=` for the camera frame) is within a few
mm of a tape-measure check. Update the firmware (5.17.0.10 recommended) before relying on the IMU.

**Placeholders:** the arm model (the B-004 arm, until the new arm is imported), the arm's mount on
the rover, the camera's mount on the end effector, and the marker positions on the keyboard.

Tests:
```bash
cd ros2_ws/src/perception/arm_vision   # inside pixi shell
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q test   # (pytest 9 vs the launch_testing plugin, as in people-pose.md)
cd ../../arm_description && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q test
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
- **`keyboard seen but no TF base_link <- arm_cam_color_optical_frame`:** nothing publishes the
  arm's joint angles. Start `arm_bringup`'s `arm.launch.py` (`/arm/joint_states`), use
  `arm_tf:=zero` on the bench, or `target_frame:=` for the camera frame. `ros2 run tf2_tools
  view_frames` shows where the tree breaks.
- **Keyboard `rejected ...: reprojection error`:** the layout doesn't match the markers: measure
  their positions again (centres, in the keyboard frame), check each `size:` and that none moved.
  `rejected ...: 0 layout marker(s)`: the ids in view aren't the layout's (10-13), or `aruco_node`'s
  `marker_ids` filters them out.
- **The keyboard jumps when the arm moves:** the arm's TF is wrong (joint zero offsets, directions,
  the camera mount): with the keyboard still, `base_link -> keyboard` should not move with the arm.
  That's the check for the URDF and `arm_axes.yaml`.
- **`mode:=sim` from a shell on the Jetson aborts with `Couldn't open X display`** (Gazebo, and the
  whole launch with it): Gazebo's camera sensor renders through GLX even with `gui:=false`. Start
  it as the dashboard and `tooling/sim-up` do: `xvfb-run -a vglrun -d egl ros2 launch ...`.
- **`Hardware Notification: Motion Module force pause ... Hardware Error`** at startup: seen once
  on firmware 5.15.1; `/arm_cam/imu` still published at 200 Hz. If the IMU stops, update the
  firmware.
- **`parameter 'serial_no' has invalid type`:** pass it through `arm_camera.launch.py serial_no:=`,
  which quotes it; a bare numeric serial in a YAML file becomes a number.
