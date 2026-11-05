# People / pose: ZED 2i + OpenPose

The rover finds people in the ZED 2i image with CMU OpenPose on the Jetson's GPU, places their
joints in 3D relative to `base_link`, and turns arm poses into gesture commands. This is a ROS 2
port of [stereolabs/zed-openpose](https://github.com/stereolabs/zed-openpose). It runs only in the
Jetson `l4t` container, because only that image has the ZED SDK and OpenPose. Other machines build
just the tests.

```mermaid
flowchart LR
    A["ZED 2i"] --> B["ZED SDK: image + depth (GPU)"]
    B -- "left image 720×405" --> C["OpenPose BODY_25 (GPU)"]
    B -- "XYZ per pixel" --> D["2D → 3D lift"]
    C -- "keypoints (u, v, score)" --> D
    D --> E["TF: camera → base_link"]
    E --> F["/people/skeletons, /people/markers"]
    C --> G["/people/debug_image"]
    F --> H["gesture_node"] --> I["/people/gesture"]
```

## Run

In the `l4t` container:

```bash
cd ros2_ws
src/perception/zed_openpose/scripts/get_openpose_models.sh      # once: ~300 MB into ros2_ws/models/openpose
pixi run build --packages-select people_msgs zed_openpose people robot_description
ros2 launch people people_rover.launch.py
```

| Arg | Default | |
|---|---|---|
| `standalone` | `true` | Also start `robot_state_publisher` + `foxglove_bridge`. Set `false` when `real.launch.py foxglove:=true` is already running them. |
| `svo_path` | `""` | Replay a ZED `.svo2` recording instead of the live camera |
| `model_pose` | `BODY_25` | `COCO` needs its own model files |
| `target_frame` | `base_link` | frame of all 3D output |

**View it:** on the ground station, open Foxglove → `ws://<rover-ip>:8765` and import
`src/perception/people/foxglove/people_rover.json`. In RViz, use fixed frame `base_link`,
MarkerArray `/people/markers`, and Image `/people/debug_image`.

**ZED udev rule:** install this once on the Jetson **host**:

```bash
docker run --rm <image> cat /etc/udev/rules.d/99-slabs.rules | sudo tee /etc/udev/rules.d/99-slabs.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The first skeleton takes 30–90 s after launch, while CUDA libraries load lazily. Later frames
are fast.

## Code

| What | Path (under `ros2_ws/src/perception/`) |
|---|---|
| Camera + OpenPose node (C++) | `zed_openpose/src/zed_openpose_node.cpp` |
| 2D → 3D math (header-only, unit-tested) | `zed_openpose/include/zed_openpose/skeleton_3d.hpp` |
| Node params | `zed_openpose/config/zed_openpose.yaml` |
| Gesture rules + node (Python) | `people/people/gestures.py`, `gesture_node.py`, `people/config/gestures.yaml` |
| Messages | `people_msgs/msg/` (`Skeleton`, `Skeletons`, `Gesture`) |
| Launch | `people/launch/people_rover.launch.py` |

## Topics

| Topic | Type | |
|---|---|---|
| `/people/skeletons` | `people_msgs/Skeletons` | Published every frame, even with nobody in view. For each person: `u[]`, `v[]`, `confidence[]` (2D px), `joints[]` (3D m in `base_link`, NaN if unknown), `centroid`. |
| `/people/markers` | `MarkerArray` | One red line list per person |
| `/people/debug_image` (+ `/compressed`) | `Image` | OpenPose rendering. Use `/compressed` over Wi-Fi. |
| `/people/gesture` | `people_msgs/Gesture` | `gesture`, `command`, `confidence`, `position` |
| `/people/gesture_marker` | `Marker` | text label above the person |
| `/zed/points`, `/zed/depth_image` | | off by default (`publish_cloud`, `publish_depth`), heavy |

## How it works

- **Two threads.** One grabs a ZED frame and prepares the input; the other runs OpenPose and
  publishes. A frame is grabbed only when OpenPose takes the previous one, so latency stays at
  about one inference.
- **2D → 3D.** Each keypoint with score ≥ `render_threshold` is looked up in the ZED XYZ map at
  the same pixel. If that pixel has no depth, the node searches outward up to 9 px. A limb is
  dropped if it's longer than 1.0 m or its midpoint is more than 1.8 m from the centroid, which
  catches joints that picked up background depth.
- **Frames.** The ZED reports in ROS axes (`RIGHT_HANDED_Z_UP_X_FWD`) relative to the left lens.
  The node publishes a static `camera_link → zed_left_camera_frame` transform (+0.06 m y) and
  transforms joints to `base_link` with TF at the image timestamp. If there's no TF, 2D is still
  published and 3D is NaN.
- **Gestures** (`gesture_node`). Only the person closest to the rover counts. Classification uses
  2D keypoints only, measured relative to the neck and scaled by body size. If any arm joint has
  confidence below 0.2, the result is `none`. A gesture must repeat `hold_frames` times in a row
  before it is reported. Rules, first match wins:

  | Gesture | Rule | Default command |
  |---|---|---|
  | `arms_u` | both wrists well above the shoulders, forearms vertical | `stop` |
  | `t_pose` | both arms level and straight out | `come_here` |
  | `right_arm_up` | right wrist above the head, left arm down | `follow_me` |
  | `left_arm_up` | left wrist above the head, right arm down | `resume` |

  **Nothing acts on `command` yet.** The commands are placeholders in `gestures.yaml`.

## Parameters

| Param | Default | Effect |
|---|---|---|
| `net_resolution` | `320x240` | Bigger finds smaller or more distant people, but is slower. Use multiples of 16. |
| `depth_mode` | `ULTRA` | `NEURAL_LIGHT` has fewer depth holes and costs more GPU |
| `camera_resolution` | `HD720` | Higher costs USB bandwidth. The OpenPose input stays 720×405. |
| `render_threshold` | `0.12` | Keypoints below this are neither drawn nor lifted |
| `conf_threshold` (gesture) | `0.2` | Arm joints below this give `none` |
| `hold_frames` (gesture) | `2` | Higher is steadier but slower to react |

## The Jetson image

`docker/Dockerfile.l4t-humble` starts from NVIDIA's Isaac ROS Humble image (L4T 36.4: CUDA 12.6,
cuDNN 9, TensorRT 10.3, OpenCV 4.5.4). It adds the ZED SDK 5.4 and OpenPose, which
`docker/build_openpose.sh` builds from source into `/opt/openpose`. Set `OPENPOSE_JOBS=2` on the
host before rebuilding the container if `nvcc` runs out of RAM.

OpenPose (pinned at `5c5d965`) needs these patches for current CUDA and protobuf:

| Patch | Why |
|---|---|
| `-std=c++11/14` → `c++17` | CUDA 12+ Thrust needs ≥ C++14 |
| `cudaDeviceProp` `clockRate`/`deviceOverlap`/`kernelExecTimeoutEnabled` → constants | Deprecated in CUDA 12, removed in CUDA 13 |
| `SetTotalBytesLimit(a, b)` → `(a)` | Newer protobuf removed the 2-arg form |
| `CPATH=/usr/local/cuda/include/cccl` | CUDA 13 only |
| `CUDA_ARCH_BIN=87` | Orin only. Cuts build time. |

OpenPose's CMake model download is disabled because CMU's model server is offline. Models come
from `get_openpose_models.sh` instead.

Traps in the Isaac ROS base:

| Trap | Symptom | Fix |
|---|---|---|
| `dl.yarnpkg.com` apt source with a dead key | every `apt-get update` exits 100 | Dockerfile deletes `yarn.list` |
| Jetson repo offers `libopencv-dev` 4.8 over the base's 4.5.4 | CMake fails on `libopencv_core.so.4.8.0` | don't install `libopencv-dev`; use the base's 4.5.4 |
| cuDNN runtime only | `cuDNN not found` | dev + header packages pinned to the runtime version |
| protobuf 26 in `/usr/local`; Boost 1.80 over `/usr` | Caffe picks the wrong protobuf; `undefined reference to boost::filesystem::path::append_v3` | `build_openpose.sh`: `/usr` first on `CMAKE_PREFIX_PATH` and `CPATH`, `Boost_DIR` → 1.80 |

Dev container problems, already fixed:

- **UID remap.** VS Code remaps `ros_user`'s uid to your host uid, so `/usr/local/zed` (owned by
  uid 1000) became unreadable and the build silently skipped `zed_openpose`. The Dockerfile now
  makes it world-readable, and makes `settings/` and `resources/` writable.
- **GPU groups.** `docker exec` terminals only get groups listed in the container's `/etc/group`,
  and the host GPU device groups (video, render, ...) aren't there, so CUDA failed with error 100
  or 801. `docker/gpu_groups.sh` runs on container start and adds `ros_user` to each GPU device's
  group.
- **Two GLibs.** OpenPose pulls in the system GLib 2.72 and pixi brings 2.88. Mixing them gave
  `libgio-2.0.so.0: undefined symbol g_variant_builder_...`. `zed_openpose/CMakeLists.txt` links
  pixi's GLib first with `-Wl,--no-as-needed`.

## Status

The current L4T 36.4 image has **not** been built or tested yet. The node, the 3D math, the
gestures, and the unit tests (`test_skeleton_3d`, plus 24 gesture cases) passed on 2026-11-02 with
an earlier JetPack 7.2 image (CUDA 13.2) on the dev Orin Nano. That run had no camera. Measured
OpenPose speed on the Nano was about 160 ms per frame (~6 fps). **Nothing has been tested with a
ZED connected** (live depth, 3D accuracy, gestures on real people, end-to-end rate).

```bash
pixi run -e l4t colcon test --packages-select zed_openpose people
pixi run -e l4t colcon test-result --verbose
```

Limitations: depth error grows with distance squared. Joints over textureless surfaces may stay
NaN. The gesture rules assume an upright person and a level camera. There's no person tracking
across frames.

Next steps: record an `.svo2` with the camera so you can develop with `svo_path` away from the
rover; measure fps on the AGX Orin; write the node that acts on `/people/gesture`.

## Troubleshooting

| Symptom | Fix |
|---|---|
| `ZED open failed: CAMERA NOT DETECTED` | No camera (this is expected without one), a USB 2 port, or no udev rule. Check `lsusb \| grep 2b03` on the host. |
| `CUDA error 100` / `801`, `NvRmMemInitNvmap failed` | GPU groups missing: `docker exec -u root <container> gpu_groups.sh`, or rebuild the container |
| `OpenPose: ... prototxt ... not found` | Run `get_openpose_models.sh` |
| `No TF base_link <- zed_left_camera_frame` | `robot_state_publisher` isn't running. Use `standalone:=true` or start the drivetrain. |
| `libgio-2.0.so.0: undefined symbol` | The GLib link lines in `zed_openpose/CMakeLists.txt` were changed. Restore them. |
| `zed_openpose` "skipping the node" at build | ZED SDK or OpenPose not visible. This is normal off the Jetson image. On the Jetson, check that `/usr/local/zed` is readable. |
| Skeletons offset from the real person | `camera_joint` in `robot_description/urdf/camera.urdf.xacro` doesn't match the real mount |
