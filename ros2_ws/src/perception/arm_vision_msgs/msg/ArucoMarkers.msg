# Every ArUco marker found in one RealSense colour frame (aruco_node -> /arm_cam/aruco/markers).
# Published for every processed frame, with an empty list when nothing is seen.

# frame_id: the colour camera's optical frame (arm_cam_color_optical_frame); stamp: the colour
# image's stamp.
std_msgs/Header header

# ArUco dictionary the ids belong to, e.g. "DICT_5X5_50".
string dictionary

# Size of the image the corners refer to.
uint32 image_width
uint32 image_height

ArucoMarker[] markers
