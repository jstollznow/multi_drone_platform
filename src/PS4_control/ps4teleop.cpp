#include "ros/ros.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "multi_drone_platform/inputAPI.h"
#include "../objects/rigidBody.h"

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
        static void input_callback(const sensor_msgs::Joy::ConstPtr& msg);
        static void construct_msg(multi_drone_platform::inputAPI& api_msg, const sensor_msgs::Joy::ConstPtr& msg);
};

void PS4_remote::construct_msg(multi_drone_platform::inputAPI& api_msg,const sensor_msgs::Joy::ConstPtr& msg)
{
    float velocity[3] = { };
    if (msg->axes[7] != 0)
    {
        // change drone id up or down
        // set drone id of message
        api_msg.msg_type = "id change";
        // will need to disable remote for a bit
        // will need to take care of drone that is being controlled before change
        // maybe hover at positon?
    }
    // check if takeoff command
    if (msg->buttons[0] == 1)
    {
        api_msg.msg_type = "take-off";
    }

    // check if landing command
    if (msg->buttons[1] == 1)
    {
        api_msg.msg_type = "land";
    }
    rigidBody myBody=rigidBody("Steven");

}

void PS4_remote::input_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    multi_drone_platform::inputAPI api_msg;
    construct_msg(api_msg,msg);
}
void PS4_remote::run(int argc, char **argv)
{
    ros::init(argc,argv,"ps4_remote");
    ros::NodeHandle n;
    input = n.subscribe<sensor_msgs::Joy>(INPUT_TOP, 10, &PS4_remote::input_callback);

    ros::Rate loop_rate(10);
}


int main(int argc, char **argv)
{
    printf("%d: ", argc);
    for (int i = 0; i < argc; i++) {
        printf("%s ", argv[i]);
    }
    printf("\n");
    PS4_remote myRemote;
    myRemote.run(argc,argv);

    return 0;
}