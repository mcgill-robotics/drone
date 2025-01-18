uint8 HOLD=0
uint8 GOTO_ACTION=1
uint8 TAKEOFF_ACTION=2
uint8 LAND=3

uint8 action_type

# arguments

# hold args
float64 hold_time_ms


# goto args
float64 n
float64 e
float64 d

float64 vn
float64 ve
float64 vd

uint8 IN_FW=0
uint8 IN_QC=1

uint8 vtol_config

# takeoff args
float64 takeoff_height
