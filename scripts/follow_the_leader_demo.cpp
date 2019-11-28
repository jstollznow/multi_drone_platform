#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "sensor_msgs/Joy.h"
#include <vector>
#include "../include/user_api.h"

#define INPUT_TOP "/ps4"
#define SERVER_FREQ 10
#define UPDATE_RATE 10

#define TAKEOFF_TIME 3.0f
#define GOTO_HOME 4.0f

// LIMIT LEVEL
// 0 DEMO - SAFE
// 1 MORE CONTROL
#define LIMIT_LEVEL 1


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
    bool localCoord;
    bool hlCommand;
    bool should_finish = false;

    bool sync;
    float ltMax;
    float rtMax;

    float max_yaw;
    float max_x;
    float max_y;
    float max_rise;
    float max_fall;


    std::array<double, 3> axesInput;
    ros::NodeHandle* myNode;
    ros::Subscriber sub;
    ros::CallbackQueue myQueue;
    ros::AsyncSpinner* mySpin;
    void defineLimits();
    void run(int argc, char **argv);
    void input_callback(const sensor_msgs::Joy::ConstPtr& msg);
    void commandHandle(const sensor_msgs::Joy::ConstPtr& msg);
    void controlUpdate();
    void resetInput();
    void terminate();

    bool emergencyHandle(int allDrones, int oneDrone);
    bool optionChangeHandle(float idChange, int coordChange);
    bool hlCommandHandle(int takeoff, int land, int hover, int goToHome);
    bool lastInputHandle(float xAxes, float yAxes, float zUpTrigger, float zDownTrigger, float yawAxes);
};

void PS4_remote::resetInput()
{
    lastInput.axesInput = {0.0f, 0.0f, 0.0f};
    lastInput.lastUpdate = ros::Time::now();
    lastInput.yaw = 0.0f;
}

bool PS4_remote::emergencyHandle(int allDrones, int oneDrone)
{
    

    if (allDrones == 1)
    // PS Button
    {   
        hlCommand = true;
        ROS_INFO("ALL EMERGENCY butt");
        for(size_t i = 0; i < drones.size(); i++)
        {
            mdp_api::cmd_emergency(drones[i]);
        }
        return true;
    }
    else if (oneDrone == 1)
    // Share Button
    {
        hlCommand = true;
        ROS_INFO("%s: EMERGENCY butt", drones[droneID].name.c_str());
        if (drones.size() > droneID)
        {
           mdp_api::cmd_emergency(drones[droneID]);
        }
        return true;
    }

    return false;
    
    
}
bool PS4_remote::optionChangeHandle(float idChange, int coordChange)
{
    
    if (coordChange != 0)
    {
        hlCommand = true;   
        if (localCoord) 
        {
            localCoord = false;
            ROS_INFO("%s: Changing control to POS", drones[droneID].name.c_str());
        }
        else
        {
            localCoord = true;
            ROS_INFO("%s: Changing control to VEL", drones[droneID].name.c_str());
        }
        return true;
    }

    return false;
}

bool PS4_remote::hlCommandHandle(int takeoff, int land, int hover, int goToHome)
{
    
 
    if (takeoff == 1)
    // cross
    {
        hlCommand = true;
        ROS_INFO("%s: Takeoff butt", drones[droneID].name.c_str());
        mdp_api::cmd_takeoff(drones[droneID], 0.5f, TAKEOFF_TIME);
        return true;
    }
    else if (land == 1)
    // circle
    {
        // hlCommand = true;
        // ROS_INFO("%s: Land butt", drones[droneID].name.c_str());
        // mdp_api::cmd_land(drones[droneID]);
        // return true;
    }
    else if (hover == 1)
    // triangle
    {
        hlCommand = true;
        ROS_INFO("%s: Hover butt", drones[droneID].name.c_str());
        mdp_api::cmd_hover(drones[droneID]);
        return true;
    }
    else if (goToHome == 1)
    // square
    {
        hlCommand = true;
        ROS_INFO("%s: GoToHome butt", drones[droneID].name.c_str());
        should_finish = true;
        return true;
    }

    return false;
    
    
}

bool PS4_remote::lastInputHandle(float xAxes, float yAxes, float zUpTrigger, float zDownTrigger, float yawAxes)
{    
    // Left Joystick (Top/Bottom)
    float x = max_x*xAxes*2.0;

    // Left Joystick (Left/Right)
    float y = max_y*yAxes*2.0;
    
    // Right Joystick (Top/Bottom)
    lastInput.yaw =max_yaw*yawAxes*2.0;

    // Triggers
    // LT go down RT go up
    float z = 0.0;

    lastInput.axesInput = {x, y, z};
    lastInput.lastUpdate = ros::Time::now();
}
void PS4_remote::commandHandle(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (sync)
    {
        // drones = mdp_api::get_all_rigidbodies();
        
        if (!emergencyHandle(msg->buttons[10], msg->buttons[8]))
        {
            if (!optionChangeHandle(msg->axes[7], msg->buttons[9]))
            {
                hlCommandHandle(msg->buttons[0], msg->buttons[1], msg->buttons[2], msg->buttons[3]);   
            }     
        }

        lastInputHandle(msg->axes[1], msg->axes[0], msg->axes[5], msg->axes[2], msg->axes[4]);

    }
    else
    {
        // sync triggers
        ltMax = std::max(msg->axes[5], ltMax);
        rtMax = std::max(msg->axes[2], rtMax);
        if (rtMax == 1.0 && ltMax == 1.0)
        {
            sync = true;
            ROS_INFO("Ready for take off");
        }
    }
   
}

void PS4_remote::controlUpdate()
{

    if (lastInput.axesInput[0] != 0.0f || lastInput.axesInput[1] != 0.0f || lastInput.axesInput[2] != 0.0f || lastInput.yaw != 0.0f)
    {
        ROS_INFO("Control update: %d", lastInput.lastUpdate.nsec);
        hlCommand = false;
        if (!localCoord)
        {
            ROS_INFO("%s: Change position by [%.2f, %.2f, %.2f] and yaw by %f", drones[droneID].name.c_str(),
            lastInput.axesInput[0], lastInput.axesInput[1], lastInput.axesInput[2], lastInput.yaw);
            mdp_api::position_msg posMsg;
            posMsg.duration =  2.0f;
            posMsg.keep_height = true;
            posMsg.relative = true;
            posMsg.position = lastInput.axesInput;
            posMsg.yaw = lastInput.yaw;
            mdp_api::set_drone_position(drones[droneID],posMsg);
        }
    }
    else
    {
        if (!hlCommand)
        {
            mdp_api::cmd_hover(drones[droneID]);
        }
    }
}

void PS4_remote::input_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
    commandHandle(msg);
}
void PS4_remote::defineLimits()
{
    switch(LIMIT_LEVEL)
    {
        // DEMO
        case 0:
            max_yaw = 0.0f;
            max_x = 0.75f;
            max_y = 0.75f;
            max_rise = 0.5f;
            max_fall = 0.25f;
            break;
        // TEST
        case 1:
            max_yaw = 5.0f;
            max_x = 1.0f;
            max_y = 1.0f;
            max_rise = 1.0f;
            max_fall = 0.5f;
            break;
    }
    
}
void PS4_remote::run(int argc, char **argv)
{    
    ros::Rate loop_rate(UPDATE_RATE);
    ROS_INFO("Initialised PS4 Remote");
    int count = 0;
 
    sync = false;
    hlCommand = true;
    localCoord = false;

    ltMax = 0.0f;
    rtMax = 0.0f;
 
    defineLimits();
 
    sub = myNode->subscribe<sensor_msgs::Joy>(INPUT_TOP, 1, &PS4_remote::input_callback);
 
    ROS_INFO("Please sync remote by pressing the left and right triggers...");
 
    mySpin->start();
    auto Drones = mdp_api::get_all_rigidbodies();
    drones = mdp_api::get_all_rigidbodies();
    PS4_remote::droneID = Drones[0].numeric_id;
    auto Follower_Drone = Drones[1];
    mdp_api::cmd_takeoff(Follower_Drone, 1.0f, 2.0f);
    mdp_api::sleep_until_idle(Follower_Drone);
    mdp_api::cmd_hover(Drones[1]);
    if (Drones.size() >= 2) {
        while (ros::ok() && !should_finish)
        {
            ros::spinOnce();
            if (sync)
            {
                controlUpdate();
                auto CurrentPos = mdp_api::get_position(Drones[0]);
                mdp_api::position_msg msg;
                msg.position = {CurrentPos.x, CurrentPos.y, 0.0f};
                msg.keep_height = true;
                msg.relative = false;
                msg.duration = 1.0f;
                msg.yaw = 0.0f;
                mdp_api::set_drone_position(Follower_Drone, msg);
                loop_rate.sleep();
                ++count;
            }
        }
        mdp_api::goto_home(Drones[0], GOTO_HOME);
        mdp_api::goto_home(Drones[1], GOTO_HOME);
        mdp_api::sleep_until_idle(Drones[0]);
        mdp_api::sleep_until_idle(Drones[1]);
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

    PS4_remote::droneID = 0;
    PS4_remote::drones = mdp_api::get_all_rigidbodies();

    while (PS4_remote::drones[PS4_remote::droneID].name.find("object") != std::string::npos)
    {
        PS4_remote::droneID++;
        if (PS4_remote::droneID >= PS4_remote::drones.size())
        {
            ROS_ERROR("No Controllable Rigid Bodies");
            start = false;
        }
    }
    ROS_INFO("Controlling %s", PS4_remote::drones[PS4_remote::droneID].name.c_str());

    PS4_remote::myNode = new ros::NodeHandle("PS4_remote");

    PS4_remote::myNode->setCallbackQueue(&PS4_remote::myQueue);
    PS4_remote::resetInput();

    if (start) 
    {
        PS4_remote::run(argc,argv);
    }
    
    delete PS4_remote::myNode;
    delete PS4_remote::mySpin;
    
    mdp_api::terminate();

    return 0;
}