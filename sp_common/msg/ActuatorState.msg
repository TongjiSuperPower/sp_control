# This message is for visualizing the ActuatorState
# The data from real Actuator will be parsed by sp_hw
# notice that this is a vector of ActuatorState

time[] stamp
string[] name
string[] type
string[] bus
int32[] id
bool[] halted

# the direct feedback from rotor's encoder
uint16[] position_raw
int16[] velocity_raw
int64[] circle
uint16[] last_position_raw

float64[] position
float64[] velocity
float64[] effort


float64[] cmd_effort 
float64[] exe_effort
