#include <ros/ros.h>
#include <stdlib.h>
include <geometry_msgs/PoseStamped.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void velocityCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/optitrack/Square/Pose", 1000, velocityCallback);
  ros::spin();

  return 0;
}
