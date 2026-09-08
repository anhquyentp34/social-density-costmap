# One pedestrian after map-frame velocity compensation (for EKF / prediction)

std_msgs/Header header

int32 id
string name

# Pose in navigation frame (usually map)
float64 position_x
float64 position_y
float64 position_z

# Absolute velocity in navigation frame after compensation
float64 velocity_x
float64 velocity_y
float64 velocity_z
float64 speed

float32 confidence
int8 tracking_state
int8 action_state

# Facing (skeleton); NOT motion heading
float64 facing_orientation
bool facing_valid
string facing_source

# Motion heading from compensated velocity
float64 motion_heading
bool motion_heading_valid
string motion_heading_source

bool velocity_compensation_valid
string measurement_reference
string velocity_compensation_mode

# Timing diagnostics (seconds)
float64 body_stamp
float64 odom_stamp
float64 body_odom_time_offset
