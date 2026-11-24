# Everyone OpenPose found in one ZED frame (zed_openpose_node -> /people/skeletons).

# frame_id: frame of every Skeleton.joints (base_link on the rover); stamp: the ZED image's stamp.
std_msgs/Header header

# OpenPose pose model, which fixes the keypoint order: "BODY_25", "COCO" or "MPI".
string model

# Size of the image the 2D keypoints (u, v) refer to.
uint32 image_width
uint32 image_height

Skeleton[] people
