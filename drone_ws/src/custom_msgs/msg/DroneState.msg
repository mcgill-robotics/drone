std_msgs/Header header

# may wanna add https://mavsdk.mavlink.io/main/en/cpp/api_reference/structmavsdk_1_1_telemetry_1_1_fixedwing_metrics.html at some point

uint32 command_queue_size

uint32 UNKNOWN=0
uint32 READY=1
uint32 TAKEOFF=2
uint32 HOLD=3
uint32 MISSION=4
uint32 RETURNTOLAUNCH=5
uint32 LAND=6
uint32 OFFBOARD=7
uint32 FOLLOWME=8
uint32 MANUAL=9
uint32 ALTCTL=10
uint32 POSCTL=11
uint32 ACRO=12
uint32 STABILIZED=13
uint32 RATTITUDE=14

uint32 flight_mode

bool in_air
bool armed

# These are the constants for the vtol state, each value is associated to a state
uint8 UNDEFINED=0
uint8 TRANSITION_TO_FW=1
uint8 TRANSITION_TO_MC=2
uint8 FW=3
uint8 MC=4

uint8 vtol_state


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
