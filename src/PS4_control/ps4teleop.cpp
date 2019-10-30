#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "sensor_msgs/Joy.h"
#include <vector>
#include "../../include/user_api.h"

#define INPUT_TOP "/ps4"
#define SERVER_FREQ 10
#define UPDATE_RATE 10

#define TAKEOFF_TIME 3.0f
#define GOTO_HOME 4.0f

#define max_yaw 0.0f
#define max_x 1.0f
#define max_y 1.0f
#define max_z 1.0f

namespace PS4_remote
{
    struct input{
        ros::Time lastUpdate;
        std::array<double, 3> axesInput;
        float yaw;
    };
    input lastInput;
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
    void resetInput();
    void terminate();
};
void PS4_remote::resetInput()
{
    lastInput.axesInput = {0.0f, 0.0f, 0.0f};
    lastInput.lastUpdate = ros::Time::now();
    lastInput.yaw = 0.0f;
}
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
        return;
    }
    // up or down D-Pad
    // id change
    if (msg->axes[7] != 0)
    {
        ROS_INFO("ID Change butt");
        ROS_INFO("%s: Hover", drones[droneID].name.c_str());
        mdp_api::cmd_hover(drones[droneID]);
        float dPadInput = msg->axes[7];
        droneID+= dPadInput;
        if (droneID < 0) droneID = drones.size() - 1;
        if (droneID >= drones.size()) droneID = 0;
        while(drones[droneID].name.find("object") != std::string::npos)
        {
            droneID += dPadInput;
            if (droneID < 0) droneID = drones.size() - 1;
            if (droneID >= drones.size()) droneID = 0;

        }
        // make iteration circular
        ROS_INFO("%s: target drone", drones[droneID].name.c_str());
        // resetInput();
        return;
    }
    // cross
    // takeoff
    if (msg->buttons[0] == 1)
    {
        ROS_INFO("%s: Takeoff butt", drones[droneID].name.c_str());
        mdp_api::cmd_takeoff(drones[droneID], 0.5f, TAKEOFF_TIME);
        // resetInput();
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
        mdp_api::goto_home(drones[droneID], GOTO_HOME);
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

    // RJ (T)
    lastInput.yaw =max_yaw*(msg->axes[4]);
    
    // LJ (T)
    float x = max_x*(msg->axes[1]);

    // RJ (L)
    float y = max_y*(msg->axes[3]);
    
    // Triggers
    // LT go down RT go up
    // ROS_INFO("Raw LT: %f RT: %f, fixed: %f",msg->axes[2], msg->axes[5], (std::min(msg->axes[2], 0.0f))-(std::min(msg->axes[5], 0.0f)));
    float z = (max_z)*(std::min(msg->axes[2], 0.0f))-(std::min(msg->axes[5], 0.0f));

    lastInput.axesInput = {x, y, z};
    lastInput.lastUpdate = ros::Time::now();
    // ROS_INFO("Async update: %d", lastInput.lastUpdate.nsec);
    // ROS_INFO("%s: [%.2f, %.2f, %.2f] and yaw by %f", drones[droneID].name.c_str(),
    //         lastInput.axesInput[0], lastInput.axesInput[1], lastInput.axesInput[2], lastInput.yaw);
}

void PS4_remote::controlUpdate()
{
    if (lastInput.axesInput[0] != 0.0f || lastInput.axesInput[1] != 0.0f || lastInput.axesInput[2] != 0.0f)
    {
        ROS_INFO("Control update: %d", lastInput.lastUpdate.nsec);

        if (velControl)
        {
            ROS_INFO("%s: Change velocity by [%.2f, %.2f, %.2f] and yaw by %f", drones[droneID].name.c_str(),
            lastInput.axesInput[0], lastInput.axesInput[1], lastInput.axesInput[2], lastInput.yaw);
            mdp_api::velocity_msg velMsg;
            velMsg.duration = 1.0f;
            velMsg.keep_height = true;
            velMsg.relative = true;
            velMsg.velocity = lastInput.axesInput;
            velMsg.yaw_rate = lastInput.yaw;
            mdp_api::set_drone_velocity(drones[droneID],velMsg);
        }
        else
        {
            ROS_INFO("%s: Change position by [%.2f, %.2f, %.2f] and yaw by %f", drones[droneID].name.c_str(),
            lastInput.axesInput[0], lastInput.axesInput[1], lastInput.axesInput[2], lastInput.yaw);
            mdp_api::position_msg posMsg;
            posMsg.duration =  1.0f;
            posMsg.keep_height = true;
            posMsg.relative = true;
            posMsg.position = lastInput.axesInput;
            posMsg.yaw = lastInput.yaw;
            mdp_api::set_drone_position(drones[droneID],posMsg);
        }
    }
    else
    {
        mdp_api::cmd_hover(drones[droneID]);
    }
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

    sub = myNode->subscribe<sensor_msgs::Joy>(INPUT_TOP, 1, &PS4_remote::input_callback);
    mySpin->start();
    while (ros::ok())
    {
        ros::spinOnce();
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
    bool start = true;

    PS4_remote::velControl = false;
    PS4_remote::droneID = 0;
    PS4_remote::drones = mdp_api::get_all_rigidbodies();
    while (PS4_remote::drones[PS4_remote::droneID].name.find("object") != std::string::npos)
    {
        // ROS_INFO("%s", PS4_remote::drones[PS4_remote::droneID].name.c_str());
        PS4_remote::droneID++;
        if (PS4_remote::droneID >= PS4_remote::drones.size())
        {
            ROS_ERROR("No Controllable Rigid Bodies");
            start = false;
        }
    }
    ROS_INFO("%s", PS4_remote::drones[PS4_remote::droneID].name.c_str());
    PS4_remote::myNode = new ros::NodeHandle("PS4_remote");
    PS4_remote::myNode->setCallbackQueue(&PS4_remote::myQueue);
    PS4_remote::resetInput();
    if (start) PS4_remote::run(argc,argv);

    delete PS4_remote::myNode;
    delete PS4_remote::mySpin;
    
    mdp_api::terminate();

    return 0;
}