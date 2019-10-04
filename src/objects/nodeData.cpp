#include "ros/ros.h"
struct node_data
{
    ros::NodeHandle* Node;
    ros::Publisher Pub;
    ros::Subscriber Sub;
};