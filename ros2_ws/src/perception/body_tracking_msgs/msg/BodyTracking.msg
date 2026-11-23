# Pose classification constants.
uint8 UNKNOWN=0
uint8 ARMS_UP=1
uint8 T_POSE=2

# Header from the ZED ObjectsStamped input message.
std_msgs/Header header

# One of UNKNOWN, ARMS_UP, or T_POSE.
uint8 skeleton_position

# Euclidean distance in metres from the ZED camera to the single body's
# depth-derived centroid. NaN means no single valid object was available.
float32 distance_m
