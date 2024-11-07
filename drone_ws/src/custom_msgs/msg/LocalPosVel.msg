std_msgs/Header header

# this is the origin of the local coordinates in lat (deg), long (deg), and alt (m)
float64 origin_lat
float64 origin_lon
float64 origin_alt

# This is the pos
float64 n
float64 e
float64 d

# This is the velocity
float64 vn
float64 ve
float64 vd

# this is the 3d orientation (attitude), as a quaternion
float64 x
float64 y
float64 z
float64 w

# this is the angular velocity (refer to)[https://mariogc.com/post/angular-velocity-quaternions/]
float64 roll_rad_s
float64 pitch_rad_s
float64 yaw_rad_s
