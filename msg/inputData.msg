# represents either a velocity or a position
geometry_msgs/Vector3 posvel

# for position
# x, y represent the forward direction of the drone
# z is yaw

# for vel
# x, y, z are angular velocities
geometry_msgs/Vector3 forward

# duration of command
float32 duration