#include "ros/ros.h"

#define SHUTDOWN_PARAM "mdp/should_shut_down"

int main(int argc, char** argv) {
    ros::init(argc, argv, "shutdown_drone_server");
    ROS_INFO("Sending shutdown request to drone server");
    ros::param::set(SHUTDOWN_PARAM, true);
    ros::shutdown();
    return EXIT_SUCCESS;
}
