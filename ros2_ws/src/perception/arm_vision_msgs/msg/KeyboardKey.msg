# One key of a localized keyboard (arm_vision keyboard_node, config/keyboard_layout.yaml).

# The layout's label: letters as capitals ("Q"), digits, unshifted punctuation (";"), names
# ("Space", "Enter", "Shift", "RShift", "F1"). arm_vision.keyboard.keys_for_text() maps text to
# these labels.
string label

# Centre of the key's top in Keyboard.header.frame_id. Press along -z of Keyboard.pose.
geometry_msgs/Point position

# Keycap top size (m): width along the keyboard's x (along the row), height along its y.
float32 width
float32 height
