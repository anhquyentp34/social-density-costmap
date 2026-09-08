# Body Tracking Message
# Tương tự như Object nhưng chuyên cho Body Tracking

# Body ID (tracking ID)
int32 id

# Confidence level (0-100)
float32 confidence

# 3D position [x, y, z] in meters
float32[3] position

# 3D velocity [vx, vy, vz] in m/s
float32[3] velocity

# 3D dimensions [width, height, length] in meters
float32[3] dimensions

# Tracking state
# 0 -> OFF (body not valid)
# 1 -> OK
# 2 -> SEARCHING (occlusion occurred, trajectory is estimated)
int8 tracking_state

# Action state
# 0 -> IDLE
# 1 -> MOVING
int8 action_state

# 2D Bounding box (top-left and bottom-right corners)
float32[2] bounding_box_2d_tl  # top-left [x, y]
float32[2] bounding_box_2d_br  # bottom-right [x, y]

# Keypoints 2D (array of [x, y] coordinates)
float32[] keypoints_2d  # Flattened array: [x1, y1, x2, y2, ...]

# Keypoints 3D (array of [x, y, z] coordinates)
float32[] keypoints_3d  # Flattened array: [x1, y1, z1, x2, y2, z2, ...]

# Number of keypoints
int32 num_keypoints

# Body format (18 or 34)
int8 body_format

