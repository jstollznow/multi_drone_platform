#include "ros/ros.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#define LOOP_RATE 10
#define INPUT_TOP "/ps4"
#define OUTPUT_TOP "/ps4_vel_ch"

class PS4_remote
{
    private:
        ros::Publisher output;
        ros::Subscriber input;
    public:
        void run(int argc, char **argv);
        void input_callback(const sensor_msgs::Joy::ConstPtr& msg);
};
void PS4_remote::input_callback(const sensor_msgs::Joy::ConstPtr& msg)
{

}
void PS4_remote::run(int argc, char **argv)
{
    ros::init(argc,argv,"ps4_remote");
    ros::NodeHandle n;
    input = n.subscribe<sensor_msgs::Joy>(INPUT_TOP, 10, input_callback);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        geometry_msgs::Vector3 msg;
        msg.x
    }
}


int main(int argc, char **argv)
{
    PS4_remote myRemote;
    myRemote.run(argc,argv);

    return 0;
}