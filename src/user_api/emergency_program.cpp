#include "ros/ros.h"
#include "std_msgs/Empty.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "mdp_emergency_program");
    ros::NodeHandle Node;
    ROS_INFO("Calling emergency on drone server");

    ros::Duration D(1.0);
    ros::Publisher Pub = Node.advertise<std_msgs::Empty> ("mdp_api_emergency", 10);
    std_msgs::Empty msg;
    D.sleep();
    Pub.publish(msg);
    D.sleep();
    Pub.publish(msg);

    ros::shutdown();
}