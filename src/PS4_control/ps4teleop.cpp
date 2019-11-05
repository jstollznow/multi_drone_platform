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

// LIMIT LEVEL
// 0 DEMO - SAFE
// 1 MORE CONTROL
#define LIMIT_LEVEL 0


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
};
void PS4_remote::resetInput()
{
    lastInput.axesInput = {0.0f, 0.0f, 0.0f};
    lastInput.lastUpdate = ros::Time::now();
    lastInput.yaw = 0.0f;
}
void PS4_remote::commandHandle(const sensor_msgs::Joy::ConstPtr& msg)
{
    if (sync)
    {
         drones = mdp_api::get_all_rigidbodies();

        // PS Button
        // ONE EMERGENCY
        if (msg->buttons[8])
        {
            hlCommand = true;
            ROS_INFO("%s: EMERGENCY butt", drones[droneID].name.c_str());
            if (drones.size() > droneID)
            {
                mdp_api::cmd_emergency(drones[droneID]);
            }
            return;
        }
        // Share Button
        // ALL EMERGENCY
        if (msg->buttons[10])
        {
            hlCommand = true;
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
            hlCommand = true;
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
            hlCommand = true;
            ROS_INFO("%s: Takeoff butt", drones[droneID].name.c_str());
            mdp_api::cmd_takeoff(drones[droneID], 0.5f, TAKEOFF_TIME);
            // resetInput();
            return;
        }
        // circle
        // land
        if (msg->buttons[1] == 1)
        {
            hlCommand = true;
            ROS_INFO("%s: Land butt", drones[droneID].name.c_str());
            mdp_api::cmd_land(drones[droneID]);
            return;
        }

        // triangle
        // hover
        if(msg->buttons[2] == 1) 
        {
            hlCommand = true;
            ROS_INFO("%s: Hover butt", drones[droneID].name.c_str());
            mdp_api::cmd_hover(drones[droneID]);
            return;
        }

        // square
        // goToHome
        if(msg->buttons[3] == 1) 
        {
            hlCommand = true;
            ROS_INFO("%s: GoToHome butt", drones[droneID].name.c_str());
            mdp_api::goto_home(drones[droneID], GOTO_HOME);
            return;
        }

        // change control
        // Options button
        if (msg->buttons[9])
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
            return;
        }

        // RJ (T)
        lastInput.yaw =max_yaw*(msg->axes[4]);
        
        // LJ (T)
        float x = max_x*(msg->axes[1]);

        // LJ (L)
        float y = max_y*(msg->axes[0]);
        
        // Triggers
        // LT go down RT go up
        // ROS_INFO("Raw LT: %f RT: %f, fixed: %f",msg->axes[2], msg->axes[5], (std::min(msg->axes[2], 0.0f))-(std::min(msg->axes[5], 0.0f)));
        float z = (max_fall/2.0f) * (msg->axes[2] - 1)-(max_rise/2.0f) * (msg->axes[5] - 1);

        lastInput.axesInput = {x, y, z};
        lastInput.lastUpdate = ros::Time::now();
        // ROS_INFO("Async update: %d", lastInput.lastUpdate.nsec);
        // ROS_INFO("%s: [%.2f, %.2f, %.2f] and yaw by %f", drones[droneID].name.c_str(),
        //         lastInput.axesInput[0], lastInput.axesInput[1], lastInput.axesInput[2], lastInput.yaw);
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
        if (localCoord)
        {
           
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
    // ros::init(argc,argv,"ps4_remote");
    
    ros::Rate loop_rate(UPDATE_RATE);
    ROS_INFO("Initialised PS4 Remote");
    int count = 0;
    sync = false;
    hlCommand = true;
    ltMax = 0.0f;
    rtMax = 0.0f;
    defineLimits();
    sub = myNode->subscribe<sensor_msgs::Joy>(INPUT_TOP, 1, &PS4_remote::input_callback);
    ROS_INFO("Please sync remote by pressing the left and right triggers...");
    mySpin->start();
    while (ros::ok())
    {
        ros::spinOnce();
        if (sync)
        {
            controlUpdate();
            loop_rate.sleep();
            ++count;
        }
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

    PS4_remote::localCoord = false;
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