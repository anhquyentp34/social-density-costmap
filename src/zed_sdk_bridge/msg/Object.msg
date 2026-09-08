# Detected object from ZED SDK Object Detection (multi-class)

# Tracking / instance ID
int32 id

# Object class label string (Person, Vehicle, Bag, Animal, Electronics, Fruit_Vegetable, Sport, Unknown)
string label

# Object class ID (SDK OBJECT_CLASS)
int8 label_id

# Confidence 0-100
float32 confidence

# 3D position [x, y, z] meters (camera / world frame theo header)
float32[3] position

# 3D velocity [vx, vy, vz] m/s
float32[3] velocity

# 3D dimensions [width, height, length] meters
float32[3] dimensions

# Tracking state: 0=OFF, 1=OK, 2=SEARCHING
int8 tracking_state

# Action state: 0=IDLE, 1=MOVING
int8 action_state

# 2D bounding box (image pixels)
float32[2] bounding_box_2d_tl
float32[2] bounding_box_2d_br
