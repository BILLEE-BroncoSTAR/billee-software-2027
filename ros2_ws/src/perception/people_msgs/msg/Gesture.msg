# The confirmed gesture of the person closest to the rover, published once per /people/skeletons.
# A new gesture is only reported after it was seen in `hold_frames` consecutive frames.

string NONE=none
string ARMS_U=arms_u              # both arms raised in a U: upper arms out, forearms up (goalpost) or arms straight up
string T_POSE=t_pose              # both arms straight out to the sides
string RIGHT_ARM_UP=right_arm_up  # person's right hand above their head, left arm down
string LEFT_ARM_UP=left_arm_up    # person's left hand above their head, right arm down

# frame_id: frame of `position`; stamp: stamp of the skeleton this was decided on.
std_msgs/Header header

string gesture
# What the rover should do about it, from gesture_node's `commands.<gesture>` parameters
# (config/gestures.yaml). Empty for "none" or an unmapped gesture.
string command
# Lowest OpenPose score among the joints the gesture was decided on (0..1).
float32 confidence
# Where the person is (neck, else their centroid), in header.frame_id. NaN if unknown.
geometry_msgs/Point position
