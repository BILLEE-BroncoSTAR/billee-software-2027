# One person's OpenPose body keypoints, in the order of the model named in Skeletons.model:
#   BODY_25: 0 nose, 1 neck, 2-4 right shoulder/elbow/wrist, 5-7 left shoulder/elbow/wrist,
#            8 mid hip, 9-11 right hip/knee/ankle, 12-14 left hip/knee/ankle, 15/16 right/left eye,
#            17/18 right/left ear, 19-21 left big toe/small toe/heel, 22-24 right big toe/small toe/heel
#   COCO:    0 nose, 1 neck, 2-7 as BODY_25, 8-10 right hip/knee/ankle, 11-13 left hip/knee/ankle,
#            14/15 right/left eye, 16/17 right/left ear
# Right/left are the person's own sides (a person facing the camera has their right on image left).

# 2D keypoints in the source image (pixels) and OpenPose's score per joint (0 = not detected).
float32[] u
float32[] v
float32[] confidence

# 3D keypoints in the parent Skeletons' header.frame_id (m). NaN where unknown: score below the
# render threshold, no valid depth near the pixel, or no TF.
geometry_msgs/Point[] joints

# Mean of the known 3D joints (zed-openpose's "center of gravity"); NaN if none are known.
geometry_msgs/Point centroid
