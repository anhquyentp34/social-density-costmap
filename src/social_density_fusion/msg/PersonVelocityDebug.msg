# Full per-person velocity pipeline debug (robot stationary evaluation)

std_msgs/Header header
int32 person_id
string name
int8 tracking_state
float32 confidence

# Position
float64[3] position_camera
float64[3] position_map

# Pipeline stages (map frame unless noted)
float64[3] raw_velocity_camera
float64[3] rotated_velocity_map
float64[3] robot_velocity_map
float64[3] rotational_compensation_map
float64[3] compensated_velocity_map
float64[3] filtered_velocity_map
float64[3] published_velocity_map

float64 raw_speed
float64 compensated_speed
float64 filtered_speed
float64 published_speed

# Processing flags
bool stationary_threshold_applied
bool jump_rejected
bool low_pass_applied
bool velocity_valid
string velocity_compensation_mode
string measurement_reference

float64 motion_heading
bool motion_heading_valid
float64 facing_orientation
bool facing_orientation_valid

float64 body_timestamp
float64 odom_timestamp
float64 time_offset
