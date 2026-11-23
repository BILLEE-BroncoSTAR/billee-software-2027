# A keyboard located from the ArUco markers stuck on it (arm_vision keyboard_node ->
# /arm_cam/keyboard/keys).

# frame_id: the node's target_frame (base_link: relative to the rover) or, without one, the
# camera's optical frame. stamp: the image the estimate comes from (or, while holding, now).
std_msgs/Header header

# Layout name (config/keyboard_layout.yaml `name`).
string layout

# The keyboard frame in header.frame_id: origin at the front-left corner of the key field,
# x along the rows to the right, y away from the typist, z up out of the key tops (z = 0 at the
# key tops). Also on TF as header.frame_id -> keyboard (the node's keyboard_frame).
geometry_msgs/Pose pose

# true: from markers in the latest image. false: no layout marker in view, holding the last
# estimate (hold_timeout), e.g. while the arm is too close to see the markers.
bool tracking

# When the markers were last seen.
builtin_interfaces/Time last_seen

# Layout markers the latest estimate used.
int32[] marker_ids

# RMS reprojection error of the latest estimate over every marker corner (px).
float32 reprojection_error

# true when marker depths corrected the PnP translation (aruco_node position_source "depth").
bool depth_corrected

# Every key in the layout, in header.frame_id.
KeyboardKey[] keys
