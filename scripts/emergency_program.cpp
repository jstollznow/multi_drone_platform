#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "../src/debug/logger/logger.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mdp_emergency_program");
    ros::NodeHandle node;
    ros::Publisher logPub = node.advertise<multi_drone_platform::log> ("mdp/log", 10);

    logger::post_log(logger::ERROR, "EMERGENCY", logPub, "Emergency program called");
    ros::Duration d(1.0);
    ros::Publisher pub = node.advertise<std_msgs::Empty> ("mdp_emergency", 10);
    std_msgs::Empty msg;
    pub.publish(msg);
    d.sleep();
    pub.publish(msg);

    ros::shutdown();
}