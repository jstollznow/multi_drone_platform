#include "ros/ros.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#define LOOP_RATE_HZ 100

#define OUTPUT_TOPIC "vflie/cmd_vel"
#define INPUT_TOPIC  "joy"

class virtual_joy
{
private:
  ros::Publisher output;
  ros::Subscriber input;

public:
  void inputCallback(const sensor_msgs::Joy::ConstPtr& msg);
  void run(int argc, char **argv);

};


void virtual_joy::inputCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  ROS_INFO("-> pub {%.1f}", msg->axes[0]);
  geometry_msgs::Twist twist;
  twist.linear.z = ((1.0f - (msg->axes[4])) - (1.0f - (msg->axes[3])) + 1.0f) * 30000; // values between 0 (0) and 60000 (1)
  twist.linear.x = -msg->axes[0] * 90.0f;          // values between 90 (right) and -90 (left) degrees
  twist.linear.y = msg->axes[1] * 90.0f;          // values between -90(back) and 90 (forward) degrees
  twist.angular.z = msg->axes[2] * 400.0f;        // values between 400 (left) and -400 (right) degrees per second 
  output.publish(twist);
}

void virtual_joy::run(int argc, char **argv)
{
  ros::init(argc, argv, "joyflie");

  ros::NodeHandle n;

  output = n.advertise<geometry_msgs::Twist>(OUTPUT_TOPIC, 1);

  input = n.subscribe<sensor_msgs::Joy>(INPUT_TOPIC, 1, &virtual_joy::inputCallback, this);

  ros::Rate loop_rate(LOOP_RATE_HZ);

  int count = 0;
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  printf("\nShutting down\n");
}


int main(int argc, char **argv)
{
  virtual_joy j;
  j.run(argc, argv);

  return 0;
}