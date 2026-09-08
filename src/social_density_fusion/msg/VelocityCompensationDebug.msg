# Per-person velocity compensation debug sample

std_msgs/Header header
int32 person_id
string name
int8 tracking_state
float32 confidence

string measurement_reference
string velocity_compensation_mode
bool compensation_valid

# Raw ZED velocity as published on /bodies (camera_link if CAMERA)
float64[3] raw_velocity_camera

# After rotation only: R * v_raw
float64[3] rotated_velocity_map

# Robot/camera linear velocity used for compensation (map)
float64[3] robot_linear_velocity_map
float64 robot_angular_z

# omega x r term (map)
float64[3] rotational_compensation_map

# Final absolute person velocity (map)
float64[3] final_velocity_map
float64 speed

float64 motion_heading
bool motion_heading_valid
float64 facing_orientation
bool facing_orientation_valid

float64 body_stamp
float64 odom_stamp
float64 body_odom_time_offset
