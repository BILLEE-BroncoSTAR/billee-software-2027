# CSI cameras (`camera_stream`)

Code: [`ros2_ws/src/camera_stream/`](../ros2_ws/src/camera_stream). How to connect, view and
troubleshoot the cameras is in [running.md → CSI cameras](running.md#csi-cameras).

Five Arducam IMX477 cameras on the Jetson's CSI ports. Every `rover.launch.py` (and so
`onboard_test.launch.py`) starts all of them, in both modes, whatever its arguments, through
`launch/csi_cameras.launch.py`. There is no argument to turn them off (`csi_cameras:=` is ignored
with a log line), and in `mode:=real` a missing or down CAN interface skips only the drivetrain, so
the cameras keep running. They only run in the Jetson container: elsewhere `rover.launch.py` skips
them with a log line.

## Launch files

| Launch | |
|---|---|
| `csi_cameras.launch.py` | one `csi_compressed_video_pub` node per camera, cam0 upwards |
| `camera_stream.launch.py` | the same plus a Foxglove bridge on `:8765`, to look at the cameras without the rest of the rover. Takes the same arguments. |

| `csi_cameras.launch.py` arg | Default | |
|---|---|---|
| `cameras` | `5` | how many to start, cam0 upwards |
| `source` | `argus` | `test`: a moving test pattern through the same encoders, no cameras needed |
| `width` / `height` / `fps` | `1280` / `720` / `30` | H.264 stream |
| `bitrate` | `3000000` | H.264 cap, bits/s: a busy scene never uses more |
| `min_qp` | `22` | H.264 quality floor (0–51, lower = finer): a calm scene uses only the bits this quality needs. `0` = no floor (always ~`bitrate`) |
| `jpeg_width` / `jpeg_height` / `jpeg_fps` | `640` / `360` / `10` | JPEG stream; `jpeg_fps:=0` turns it off |
| `use_sim_time` | `false` | `rover.launch.py mode:=sim` sets it |

The camera list is `config/cameras.yaml` in `camera_stream`: name and
`flip_method` per camera. cam1 `front` is mounted upside down (`2`, rotate 180°); the rest
are `0`. Other values: `4` mirror left-right, `6` mirror top-bottom, `1` / `3`
rotate 90° (the stream becomes portrait, 720×1280). The flip happens in the Jetson's video hardware
before encoding, so every viewer gets the corrected image. Edit the table and relaunch; with the
symlink install there's nothing to rebuild.

## The node (`csi_compressed_video_publisher.py`)

Per camera N (Argus `sensor-id` N, the Nth camera detected at boot in port order):

| Topic | Type | For |
|---|---|---|
| `/camN/video` | `foxglove_msgs/CompressedVideo`, H.264 Annex B, one frame per message | Foxglove's Image panel |
| `/camN/image_raw/compressed` | `sensor_msgs/CompressedImage`, JPEG | RViz's Image display (and Foxglove) |

Both are best effort, depth 2, so a slow link drops frames instead of building latency. RViz
subscribers must be best effort too. DDS only sends the JPEG stream while something subscribes.

**Pipeline.** The node runs one `gst-launch-1.0` child process per camera:
`nvarguscamerasrc` → (`nvvidconv flip-method`) → `tee` → `nvv4l2h264enc` (keyframe every 30
frames, no B-frames, an access-unit delimiter per frame) to stdout, and → `nvvidconv` →
`videorate` → `nvjpegenc` to a second pipe. The node splits the H.264 byte stream into frames at
the delimiters and the JPEG stream at the JPEG markers, and publishes them. It uses a child process
because ROS's Python comes from pixi and has no GStreamer bindings, while NVIDIA's camera and
encoder plugins are built for the system GStreamer (from `gstreamer1.0-tools` in the image). The
child runs under `setpriv --pdeathsig`, so it dies with the node, and is stopped with SIGINT so the camera closes
cleanly.

**A missing camera never stops the launch.** If `gst-launch-1.0`, the NVIDIA plugins or the Argus
socket (`/tmp/argus_socket`) is missing, the camera wasn't detected at boot, or the pipeline fails,
the node logs one warning saying why, publishes a 2 fps placeholder on both topics ("camN …: NO
CAMERA" text, or colour bars without the pango plugin), and checks again every 10 s
(`recheck_s`).

**Compression.** H.264 High profile, the encoder's Slow preset, 2 reference frames, and VBR with a
QP floor (`min_qp`, 22, close to visually lossless) capped at `bitrate`. At the same QP this is
~35% smaller than the encoder's defaults (Baseline, UltraFast), and the floor stops it spending
bits a calm scene doesn't need, where CBR padded every scene to `bitrate`. Measured on the rover
(1280×720 @ 30, indoors, 2026-11-20): 2954 → 1564 kbit/s offline, ~1100 kbit/s live; a worst-case
noise pattern stays at the 3 Mbit/s cap, at a quality no lower than the old CBR stream. Three
cameras at once still encode in real time. H.265 would be smaller again, but Foxglove on a Linux
ground station usually can't decode it.

Other node parameters: `encoding` (`h264` or `h265`), `preset_level` (4, Slow), `gop` (30), `jpeg_quality` (70),
`placeholder_fps` (2), `frame_id` (`camN_optical`), `gst_launch` (`/usr/bin/gst-launch-1.0`).

## `foxglove_msgs` (vendored)

`CompressedVideo` comes from Foxglove's message package, vendored unmodified in
[`ros2_ws/src/third_party/foxglove_msgs/`](../ros2_ws/src/third_party/foxglove_msgs) (tag
`ros-v3.6.0`; provenance in its `VENDORED.md`) and built with the rest of the workspace. The conda
package exists only for a newer RoboStack build than the one the pixi environment is locked to.
Without it the node logs `foxglove_msgs isn't built` and publishes JPEG only. The ground
station's Foxglove bridge needs it too, to forward the H.264 topics.
