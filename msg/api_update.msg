# takeoff, land, etc
string msgType

# whether the position or velocity is relative
bool relativeXY

# position or velocity
geometry_msgs/Vector3 posVel

# how long the command is valid for
float32 duration

# yaw or yaw rate depending on whether it is a velocity or position command
float32 yawVal

# whether to relative height
bool relativeZ