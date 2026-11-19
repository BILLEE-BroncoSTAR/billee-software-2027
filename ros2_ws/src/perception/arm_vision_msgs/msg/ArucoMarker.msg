# One ArUco marker in one RealSense colour frame (arm_vision aruco_node).

# Marker id in the node's dictionary (parameter `dictionary`, e.g. DICT_5X5_50).
int32 id

# Marker centre and orientation in ArucoMarkers.header.frame_id, the colour camera's optical
# frame (x right, y down, z forward, out of the lens). The orientation is OpenCV's marker frame:
# x along the marker's top edge (left to right), y up the marker, z out of its printed face
# toward the camera. It is identity when there is no orientation (has_orientation false).
geometry_msgs/Pose pose

# Where pose.position comes from:
#   "depth": the aligned depth image, median over the marker, deprojected at its centre pixel
#   "pnp":   solvePnP from the four corners and the marker's printed size (marker_size)
string position_source
string POSITION_DEPTH=depth
string POSITION_PNP=pnp

# true when pose.orientation was estimated (solvePnP ran: marker_size > 0 and it converged).
bool has_orientation

# Straight-line camera -> marker centre distance (m).
float32 distance

# Median aligned depth over the marker (m); NaN where it had no valid depth.
float32 depth

# solvePnP's RMS reprojection error over the four corners (px); NaN without a PnP solution.
# A few px or more: the marker size is wrong, the marker is bent, or the detection is poor.
float32 reprojection_error

# Corners in the colour image (px) as OpenCV returns them, clockwise from the marker's own
# top-left: u0, v0 (top-left), u1, v1 (top-right), u2, v2 (bottom-right), u3, v3 (bottom-left).
float32[8] corners
