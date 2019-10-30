#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "sensor_msgs/Joy.h"
#include <vector>
#include "../../include/user_api.h"

#define INPUT_TOP "/ps4"
#define SERVER_FREQ 10
#define UPDATE_RATE 2

#define max_yaw 50.0f
#define max_x 0.5f
#define max_y 0.5f
#define max_z 0.5f

namespace PS4_remote
{
    // struct input{
    //     ros::Header timeStamp;
    //     std::array<double, 3> axesInput;
    //     ros::
    // };
    // input lastInput;
    std::vector<mdp_api::id> drones;
    int droneID;
    bool velControl;
    std::array<double, 3> axesInput;
    ros::NodeHandle* myNode;
    ros::Subscriber sub;
    ros::CallbackQueue myQueue;
    ros::AsyncSpinner* mySpin;
    void run(int argc, char **argv);
    void input_callback(const sensor_msgs::Joy::ConstPtr& msg);
    void commandHandle(const sensor_msgs::Joy::ConstPtr& msg);
    void controlUpdate();
    void terminate();
};

void PS4_remote::commandHandle(const sensor_msgs::Joy::ConstPtr& msg)
{
    drones = mdp_api::get_all_rigidbodies();

    // PS Button
    // ONE EMERGENCY
    if (msg->buttons[10] == 1)
    {
        ROS_INFO("%s: EMERGENCY butt", drones[droneID].name.c_str());
        if (drones.size() > droneID)
        {
            mdp_api::cmd_emergency(drones[droneID]);
        }
        return;
    }
    // Share Button
    // ALL EMERGENCY
    if (msg->buttons[8])
    {
        ROS_INFO("ALL EMERGENCY butt");
        for(size_t i = 0; i < drones.size(); i++)
        {
            mdp_api::cmd_emergency(drones[i]);
        }
        for(size_t i = 0; i < drones.size(); i++)
        {
            mdp_api::cmd_emergency(drones[i]);
        }
        // subject to change
        // ros::shutdown();
        return;
    }
    // up or down D-Pad
    // id change
    if (msg->axes[7] != 0)
    {
        ROS_INFO("ID Change butt");
        ROS_INFO("%s: Hover", drones[droneID].name.c_str());
        mdp_api::cmd_hover(drones[droneID]);
        droneID+= msg->axes[7];

        // make iteration circular
        if (droneID < 0) droneID = drones.size() - 1;
        if (droneID >= drones.size()) droneID = 0;
        ROS_INFO("%s: target drone", drones[droneID].name.c_str());
        return;
    }
    // cross
    // takeoff
    if (msg->buttons[0] == 1)
    {
        ROS_INFO("%s: Takeoff butt", drones[droneID].name.c_str());
        mdp_api::cmd_takeoff(drones[droneID], 0.5, 1/UPDATE_RATE);
        return;
    }
    // circle
    // land
    if (msg->buttons[1] == 1)
    {
        ROS_INFO("%s: Land butt", drones[droneID].name.c_str());
        mdp_api::cmd_land(drones[droneID]);
        return;
    }

    // triangle
    // hover
    if(msg->buttons[2] == 1) 
    {
        ROS_INFO("%s: Hover butt", drones[droneID].name.c_str());
        mdp_api::cmd_hover(drones[droneID]);
        return;
    }

    // square
    // goToHome
    if(msg->buttons[3] == 1) 
    {
        ROS_INFO("%s: GoToHome butt", drones[droneID].name.c_str());
        mdp_api::goto_home(drones[droneID], 1/UPDATE_RATE);
        return;
    }

    // change control
    // Options button
    if (msg->buttons[9])
    {
        if (velControl) 
        {
            velControl = false;
            ROS_INFO("%s: Changing control to POS", drones[droneID].name.c_str());
        }
        else
        {
            velControl = true;
            ROS_INFO("%s: Changing control to VEL", drones[droneID].name.c_str());
        }
        return;
    }
            // LJ (L)
        // Yaw
        float yawScale = max_yaw;
        float yaw =yawScale*(msg->axes[0]);
        
        // RJ (L)
        // x
        float xScale = max_x;
        float x = xScale*(msg->axes[3]);

        // RJ (T)
        // y
        float yScale = max_y;
        float y = yScale*(msg->axes[4]);
        
        // LJ (T)
        // z
        float zScale = max_z;
        float z = zScale*(msg->axes[1]);

    ROS_INFO("This is last message %f", msg->header.stamp.nsec);

}

void PS4_remote::controlUpdate()
{
    // if (!(msg->axes[0] == 0.0 && msg->axes[1] == 0.0 && msg->axes[3] == 0.0 && msg->axes[4] == 0.0))
    // {
    //     // LJ (L)
    //     // Yaw
    //     float yawScale = max_yaw;
    //     float yaw =yawScale*(msg->axes[0]);
        
    //     // RJ (L)
    //     // x
    //     float xScale = max_x;
    //     float x = xScale*(msg->axes[3]);

    //     // RJ (T)
    //     // y
    //     float yScale = max_y;
    //     float y = yScale*(msg->axes[4]);
        
    //     // LJ (T)
    //     // z
    //     float zScale = max_z;
    //     float z = zScale*(msg->axes[1]);

    //     if (velControl)
    //     {
    //         ROS_INFO("%s: Change velocity by [%.2f, %.2f, %.2f] and yaw by %f", drones[droneID].name.c_str(),
    //         x, y, z, yaw);
    //         mdp_api::velocity_msg velMsg;
    //         velMsg.duration = (1.0f/UPDATE_RATE);
    //         velMsg.keep_height = true;
    //         velMsg.relative = true;
    //         velMsg.velocity = {x, y, z};
    //         velMsg.yaw_rate = yaw;
    //         mdp_api::set_drone_velocity(drones[droneID],velMsg);
    //     }
    //     else
    //     {
    //         ROS_INFO("%s: Change position by [%.2f, %.2f, %.2f] and yaw by %f", drones[droneID].name.c_str(),
    //         x, y, z, yaw);
    //         mdp_api::position_msg posMsg;
    //         posMsg.duration = (1.0f/UPDATE_RATE);
    //         posMsg.keep_height = true;
    //         posMsg.relative = true;
    //         posMsg.position = {x, y, z};
    //         posMsg.yaw = yaw;
    //         mdp_api::set_drone_position(drones[droneID],posMsg);
    //     }
    // }
}

void PS4_remote::input_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    commandHandle(msg);
}
void PS4_remote::run(int argc, char **argv)
{
    // ros::init(argc,argv,"ps4_remote");
    
    ros::Rate loop_rate(UPDATE_RATE);
    ROS_INFO("Initialised PS4 Remote");
    int count = 0;
    droneID = 0;

    sub = myNode->subscribe<sensor_msgs::Joy>(INPUT_TOP, 1, &PS4_remote::input_callback);
    mySpin->start();
    while (ros::ok())
    {
        ros::spinOnce();
        ROS_INFO("NORMAL LOOP");
        controlUpdate();
        loop_rate.sleep();
        ++count;
    }

}
void PS4_remote::terminate()
{
    delete myNode;
    delete mySpin;
    ROS_INFO("Shutting Down Client API Connection");
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ps4_remote");
    PS4_remote::mySpin = new ros::AsyncSpinner(1,&PS4_remote::myQueue);
    mdp_api::initialise(SERVER_FREQ);

    PS4_remote::velControl = false;
    PS4_remote::droneID = 0;

    PS4_remote::drones = mdp_api::get_all_rigidbodies();
    PS4_remote::myNode = new ros::NodeHandle("PS4_remote");
    PS4_remote::myNode->setCallbackQueue(&PS4_remote::myQueue);

    PS4_remote::run(argc,argv);

    delete PS4_remote::myNode;
    delete PS4_remote::mySpin;
    
    mdp_api::terminate();

    return 0;
}