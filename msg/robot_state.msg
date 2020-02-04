# Header to keep track of message sequence and timestamp
Header header
  uint32 seq
  time stamp
  string frame_id

# State variables for robot position in 6 DOF and their derivatives 
float64 x
float64 y
float64 z
float64 roll
float64 pitch
float64 yaw
float64 x_dot
float64 y_dot
float64 z_dot
float64 roll_dot
float64 pitch_dot
float64 yaw_dot