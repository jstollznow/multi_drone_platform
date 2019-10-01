#include "ros/ros.h"

#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "multi_drone_platform/inputAPI.h"
#include "../objects/rigidBody.h"
#include "../../include/user_api.h"
#define LOOP_RATE 10
#define INPUT_TOP "/ps4"
#define OUTPUT_TOP "/api_input"
struct node_data
{
    ros::NodeHandle* Node;
    ros::Publisher output;
    ros::Subscriber input;
} NodeData;

namespace PS4_remote
{
    static int droneID;
    void run(int argc, char **argv);
    static void input_callback(const sensor_msgs::Joy::ConstPtr& msg);
    static void construct_msg(const sensor_msgs::Joy::ConstPtr& msg);
    static void sendAPImsg(std::string msg_type, uint32_t droneID, float velX = 0.0f, float velY = 0.0f, float velZ = 0.0f, float yawRate = 0.0f);
    void terminate();
};

void PS4_remote::construct_msg(const sensor_msgs::Joy::ConstPtr& msg)
{
    geometry_msgs::Vector3 v;
    std::string msg_type = "VELOCITY";
    // PS Button
    if (msg->buttons[10] == 1) msg_type = "ELAND";
    // up or down D-Pad
    if (msg->axes[7] != 0)
    {
        // change drone id up or down
        // set drone id of message
        sendAPImsg("HOVER", droneID);
        droneID+= msg->axes[7];
        if (droneID<0) droneID = 0;
        msg_type = "ID";
        ROS_INFO("You now control drone with ID %d", droneID);
        // api_msg.msg_type = "ID";
        // will need to disable remote for a bit
        // will need to take care of drone that is being controlled before change
        // maybe hover at positon?
    }
    // cross
    if (msg->buttons[0] == 1) msg_type = "TAKE-OFF";

    // circle
    if (msg->buttons[1] == 1) msg_type = "LAND";

    // triangle
    if(msg->buttons[2] == 1) msg_type = "HOVER";

    if (msg_type!= "VELOCITY")
    {
        sendAPImsg(msg_type,droneID);
        return;
    }
    else
    {
        // LJ (L)
        // Yaw
        float yawScale = 400.0f;
        float yaw = yawScale*(msg->axes[0]);
        
        // LJ (T)
        // Thrust
        float zScale = 60000.0f;
        float z = DRONE THRUST*(msg->axes[1]);

        // RJ (L)
        // Roll
        float xScale = 90.0f;
        float x = xScale*(msg->axes[3]);

        // RJ (T)
        // Pitch
        float yScale = 90.0f;
        float y = yScale*(msg->axes[4]);

        sendAPImsg(msg_type, droneID, x, y, z, yaw);
    }
}
void PS4_remote::sendAPImsg(std::string msg_type, uint32_t droneID, float velX, float velY, float velZ, float yawRate)
{
    ROS_INFO("%s command sent to drone %d", msg_type.c_str(), droneID);

    if (strcmp(msg_type.c_str(),"VELOCITY") == 0)
    {
        ROS_INFO("with magnitude [%.2f, %.2f, %.2f, %.2f]", velX, velY, velZ, yawRate);
    }

    multi_drone_platform::inputAPI Msg;
    Msg.drone_id = droneID;
    Msg.msg_type = msg_type;

    Msg.movement.vec3.x = velX;
    Msg.movement.vec3.y = velY;
    Msg.movement.vec3.z = velZ;
    Msg.movement.yaw = yawRate;

    NodeData.output.publish(Msg);
}
void PS4_remote::input_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    construct_msg(msg);
}
void PS4_remote::run(int argc, char **argv)
{
    ros::init(argc,argv,"ps4_remote");
    NodeData.Node= new ros::NodeHandle;
    NodeData.output = NodeData.Node->advertise<multi_drone_platform::inputAPI>(OUTPUT_TOP,100);
    NodeData.input = NodeData.Node->subscribe<sensor_msgs::Joy>(INPUT_TOP, 10, &PS4_remote::input_callback);
    
    ros::Rate loop_rate(10);
    ROS_INFO("Initialised PS4 Remote");
    int count = 0;
    droneID = 0;
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    delete NodeData.Node;
}
void PS4_remote::terminate()
{
    delete NodeData.Node;
    ROS_INFO("Shutting Down Client API Connection");
}

int main(int argc, char **argv)
{
    printf("%d: ", argc);
    for (int i = 0; i < argc; i++) {
        printf("%s ", argv[i]);
    }
    printf("\n");
    PS4_remote::run(argc,argv);

    return 0;
}