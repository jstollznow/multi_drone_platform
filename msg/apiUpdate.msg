# takeoff, land, etc
string msg_type

# whether the position or velocity is relative
bool relative

# position or velocity
geometry_msgs/Vector3 posvel

# how long the command is valid for
float32 duration

# yaw or yaw rate depending on whether it is a velocity or position command
float32 yawVal

# whether to maintain constant height
bool constHeight