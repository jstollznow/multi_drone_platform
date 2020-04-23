# Multi-Drone-Platform
Our honours project drone environment, if this goes public we have made it

## Installation

follow this guide to install ROS kinetic: http://wiki.ros.org/kinetic/Installation

follow this guide to create a ROS catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace

| name | how to install | reason |
|------|---------------|-------|
| libgtkmm | `sudo apt-get install libgtkmm-3.0-dev` | used to compile debug windows |
| joy node | `sudo apt-get install ros-kinetic-joy` | used to operate ps4 teleop |
| crazyflie_ros | follow install instruction in catkin workspace https://github.com/whoenig/crazyflie_ros | used to operate the crazyflie implementation |
| natnet_ros | follow install instruction in catkin workspace https://github.com/mje-nz/natnet_ros | used to interface with Optitrack Natnet. Note: vrpn alternative exists using `vrpn_client_ros` |

then clone this repository into your catkin workspace/src folder.

Then run `catkin_make` in the base workspace folder.