#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "sensor_msgs/Joy.h"
#include <vector>
#include "user_api.h"
#include <cmath>

#define INPUT_TOP "/ps4"
#define SERVER_FREQ 10
#define UPDATE_RATE 10

#define TAKEOFF_TIME 3.0f
#define GO_TO_HOME 4.0f

#define MSG_DUR 2.0f
// LIMIT LEVEL
// 0 DEMO - SAFE
// 1 MORE CONTROL
#define LIMIT_LEVEL 0


namespace ps4_remote
{
struct input{
    ros::Time lastUpdate;
    std::array<double, 3> axesInput;
    float yaw;
};
input lastInput;
std::vector<mdp::id> drones;
int droneID;
bool localCoord;
bool highLevelCommand;

bool sync;
float ltMax;
float rtMax;

float maxYaw;
float maxX;
float maxY;
float maxRise;
float maxFall;

ros::NodeHandle* myNode;
ros::Subscriber sub;
ros::CallbackQueue myQueue;
ros::AsyncSpinner* mySpin;

void define_limits();
void run(int argc, char **argv);
void input_callback(const sensor_msgs::Joy::ConstPtr& msg);
void control_update();
void reset_input();
void terminate();

bool checkForHover();
void command_handle(const sensor_msgs::Joy::ConstPtr& msg);
bool emergency_handle(int allDrones, int oneDrone);
bool option_change_handle(float idChange, int coordChange);
bool high_lvl_command_handle(int takeoff, int land, int hover, int goToHome);
bool last_input_handle(float xAxes, float yAxes, float zUpTrigger, float zDownTrigger, float yawAxes);

}

void ps4_remote::reset_input() {
    lastInput.axesInput = {0.0f, 0.0f, 0.0f};
    lastInput.lastUpdate = ros::Time::now();
    lastInput.yaw = 0.0f;
}

bool ps4_remote::emergency_handle(int allDrones, int oneDrone) {
    

    // PS Button
    if (allDrones == 1) {   
        highLevelCommand = true;
        ROS_INFO("ALL EMERGENCY butt");
        for(size_t i = 0; i < drones.size(); i++)
        {
            mdp::cmd_emergency(drones[i]);
        }
        return true;
    }
    // Share Button
    else if (oneDrone == 1) {
        highLevelCommand = true;
        ROS_INFO("%s: EMERGENCY butt", drones[droneID].name.c_str());
        if (drones.size() > droneID) {
           mdp::cmd_emergency(drones[droneID]);
        }
        return true;
    }

    return false; 
}
bool ps4_remote::option_change_handle(float idChange, int coordChange) {
    
    // up or down D-Pad
    if ((int)idChange != 0) {
        highLevelCommand = true;
        ROS_INFO("ID Change butt");
        ROS_INFO("%s: Hover", drones[droneID].name.c_str());
        mdp::cmd_hover(drones[droneID]);
        droneID += (int)idChange;
        if (droneID < 0) droneID = drones.size() - 1;
        if (droneID >= drones.size()) droneID = 0;
        while(drones[droneID].name.find("object") != std::string::npos) {
            droneID += (int)idChange;
            if (droneID < 0) droneID = drones.size() - 1;
            if (droneID >= drones.size()) droneID = 0;

        }
        // make iteration circular
        ROS_INFO("%s: target drone", drones[droneID].name.c_str());

        return true;
    }
    // options button
    else if (coordChange != 0) {
        highLevelCommand = true;   
        if (localCoord) {
            localCoord = false;
            ROS_INFO("%s: Changing control to absolute coord", drones[droneID].name.c_str());
        }
        else {
            localCoord = true;
            ROS_INFO("%s: Changing control to relative coord", drones[droneID].name.c_str());
        }
        return true;
    }

    return false;
}

bool ps4_remote::high_lvl_command_handle(int takeoff, int land, int hover, int goToHome) {
    
    // cross
    if (takeoff) {
        highLevelCommand = true;
        ROS_INFO("%s: Takeoff butt", drones[droneID].name.c_str());
        mdp::cmd_takeoff(drones[droneID], 0.5f, TAKEOFF_TIME);
        return true;
    }
    // circle
    else if (land) {
        highLevelCommand = true;
        ROS_INFO("%s: Land butt", drones[droneID].name.c_str());
        mdp::cmd_land(drones[droneID]);
        return true;
    }
    // triangle
    else if (hover) {
        highLevelCommand = true;
        ROS_INFO("%s: Hover butt", drones[droneID].name.c_str());
        mdp::cmd_hover(drones[droneID]);
        return true;
    }
    // square
    else if (goToHome) {
        highLevelCommand = true;
        ROS_INFO("%s: GoToHome butt", drones[droneID].name.c_str());
        mdp::go_to_home(drones[droneID],GO_TO_HOME);
        return true;
    }

    return false;
}

bool ps4_remote::last_input_handle(float xAxes, float yAxes, float zUpTrigger, float zDownTrigger, float yawAxes) {    
    // Left Joystick (Top/Bottom)
    float x = maxX*xAxes * MSG_DUR;

    // Left Joystick (Left/Right)
    float y = maxY*yAxes * MSG_DUR;
    
    // Right Joystick (Top/Bottom)
    lastInput.yaw =maxYaw*yawAxes * MSG_DUR;

    // Triggers
    // LT go down RT go up
    float z = ((maxFall) / MSG_DUR) * (zDownTrigger - 1)-((maxRise) / MSG_DUR) * (zUpTrigger - 1);

    lastInput.axesInput = {x, y, z};
    lastInput.lastUpdate = ros::Time::now();
}
void ps4_remote::command_handle(const sensor_msgs::Joy::ConstPtr& msg) {
    if (sync) {
        drones = mdp::get_all_rigidbodies();
        
        if (!emergency_handle(msg->buttons[10], msg->buttons[8])) {
            if (!option_change_handle(msg->axes[7], msg->buttons[9])) {
                high_lvl_command_handle(msg->buttons[0], msg->buttons[1], msg->buttons[2], msg->buttons[3]);   
            }     
        }

        last_input_handle(msg->axes[1], msg->axes[0], msg->axes[5], msg->axes[2], msg->axes[4]);

    }
    else {
        // sync triggers
        ltMax = std::max(msg->axes[5], ltMax);
        rtMax = std::max(msg->axes[2], rtMax);
        if (rtMax && ltMax) {
            sync = true;
            ROS_INFO("Ready for take off");
        }
    }
   
}

bool ps4_remote::checkForHover () {
    float deadzone = 0.1;
    bool hoverComm = true;
    for(int i = 0; i < 3; i++) {
        if (abs(lastInput.axesInput[i]) > deadzone) {
            hoverComm = false;
            break;
        }
    }
    return hoverComm;
}

void ps4_remote::control_update() {

        ROS_INFO("Control update: %d", lastInput.lastUpdate.nsec);
        if (lastInput.axesInput[0] != 0.0f || lastInput.axesInput[1] != 0.0f || lastInput.axesInput[2] != 0.0f || lastInput.yaw != 0.0f) {
            highLevelCommand = false;
        }
        if (!highLevelCommand) {
            if (true) {
                ROS_INFO("%s: Change position by [%.2f, %.2f, %.2f] and yaw by %f", drones[droneID].name.c_str(),
                         lastInput.axesInput[0], lastInput.axesInput[1], lastInput.axesInput[2], lastInput.yaw);
                mdp::position_msg posMsg;
                posMsg.duration = MSG_DUR;
                posMsg.keepHeight = true;
                posMsg.relative = true;
                posMsg.position = lastInput.axesInput;
                posMsg.yaw = lastInput.yaw;
                mdp::set_drone_position(drones[droneID], posMsg);
            }
            else {
//                mdp::cmd_hover(drones[droneID]);
//                highLevelCommand = true;
            }
        }
}

void ps4_remote::input_callback(const sensor_msgs::Joy::ConstPtr& msg) {
    command_handle(msg);
}
void ps4_remote::define_limits() {
    switch(LIMIT_LEVEL) {
        // DEMO
        case 0:
            maxYaw = 0.0f;
            maxX = 0.75f;
            maxY = 0.75f;
            maxRise = 1.0f;
            maxFall = 0.5f;
            break;
        // TEST
        case 1:
            maxYaw = 5.0f;
            maxX = 1.0f;
            maxY = 1.0f;
            maxRise = 1.0f;
            maxFall = 0.5f;
            break;
    }
    
}
void ps4_remote::run(int argc, char **argv) {    
    ros::Rate loop_rate(UPDATE_RATE);
    ROS_INFO("Initialised PS4 Remote");
    int count = 0;
 
    sync = false;
    highLevelCommand = true;
    localCoord = false;

    ltMax = 0.0f;
    rtMax = 0.0f;
 
    define_limits();
 
    sub = myNode->subscribe<sensor_msgs::Joy>(INPUT_TOP, 1, &ps4_remote::input_callback);
 
    ROS_INFO("Please sync remote by pressing the left and right triggers...");
 
    mySpin->start();
 
    while (ros::ok()) {
        ros::spinOnce();
        if (sync) {
            control_update();
            loop_rate.sleep();
            ++count;
        }
    }

}
void ps4_remote::terminate() {
    delete myNode;
    delete mySpin;
    ROS_INFO("Shutting Down Client API Connection");
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"ps4_remote");
    ps4_remote::mySpin = new ros::AsyncSpinner(1,&ps4_remote::myQueue);
    mdp::initialise(SERVER_FREQ, "ps4teleop");
    bool start = true;

    ps4_remote::droneID = 0;
    ps4_remote::drones = mdp::get_all_rigidbodies();

    while (ps4_remote::drones[ps4_remote::droneID].name.find("object") != std::string::npos) {
        ps4_remote::droneID++;
        if (ps4_remote::droneID >= ps4_remote::drones.size()) {
            ROS_ERROR("No Controllable Rigid Bodies");
            start = false;
        }
    }
    ROS_INFO("Controlling %s", ps4_remote::drones[ps4_remote::droneID].name.c_str());

    ps4_remote::myNode = new ros::NodeHandle("PS4_remote");

    ps4_remote::myNode->setCallbackQueue(&ps4_remote::myQueue);
    ps4_remote::reset_input();

    if (start) {
        ps4_remote::run(argc,argv);
    }
    
    delete ps4_remote::myNode;
    delete ps4_remote::mySpin;
    
    mdp::terminate();

    return 0;
}