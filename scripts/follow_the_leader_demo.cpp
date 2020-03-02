#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "sensor_msgs/Joy.h"
#include <vector>
#include "user_api.h"

#define INPUT_TOP "/ps4"
#define SERVER_FREQ 10
#define UPDATE_RATE 10

#define TAKEOFF_TIME 3.0f
#define GO_TO_HOME 4.0f

// LIMIT LEVEL
// 0 DEMO - SAFE
// 1 MORE CONTROL
#define LIMIT_LEVEL 1


namespace ps4_remote {
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
bool shouldFinish = false;

bool sync;
float ltMax;
float rtMax;

float maxYaw;
float maxX;
float maxY;
float maxRise;
float maxFall;


std::array<double, 3> axesInput;
ros::NodeHandle* myNode;
ros::Subscriber sub;
ros::CallbackQueue myQueue;
ros::AsyncSpinner* mySpin;
void define_limits();
void run(int argc, char **argv);
void input_callback(const sensor_msgs::Joy::ConstPtr& msg);
void command_handle(const sensor_msgs::Joy::ConstPtr& msg);
void control_update();
void reset_input();
void terminate();

bool emergency_handle(int allDrones, int oneDrone);
bool option_change_handle(float idChange, int coordChange);
bool high_lvl_command_handle(int takeoff, int land, int hover, int goToHome);
bool last_input_handle(float xAxes, float yAxes, float zUpTrigger, float zDownTrigger, float yawAxes);
};

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
    
    if (coordChange != 0) {
        highLevelCommand = true;   
        if (localCoord) {
            localCoord = false;
            ROS_INFO("%s: Changing control to POS", drones[droneID].name.c_str());
        }
        else {
            localCoord = true;
            ROS_INFO("%s: Changing control to VEL", drones[droneID].name.c_str());
        }
        return true;
    }

    return false;
}

bool ps4_remote::high_lvl_command_handle(int takeoff, int land, int hover, int goToHome) { 
    // cross
    if (takeoff == 1) {
        highLevelCommand = true;
        ROS_INFO("%s: Takeoff butt", drones[droneID].name.c_str());
        mdp::cmd_takeoff(drones[droneID], 0.5f, TAKEOFF_TIME);
        return true;
    }
    // circle
    else if (land == 1) {
        // hlCommand = true;
        // ROS_INFO("%s: Land butt", drones[droneID].name.c_str());
        // mdp_api::cmd_land(drones[droneID]);
        // return true;
    }
    // triangle
    else if (hover == 1) {
        highLevelCommand = true;
        ROS_INFO("%s: Hover butt", drones[droneID].name.c_str());
        mdp::cmd_hover(drones[droneID]);
        return true;
    }
    // square
    else if (goToHome == 1) {
        highLevelCommand = true;
        ROS_INFO("%s: GoToHome butt", drones[droneID].name.c_str());
        shouldFinish = true;
        return true;
    }

    return false;
    
    
}

bool ps4_remote::last_input_handle(float xAxes, float yAxes, float zUpTrigger, float zDownTrigger, float yawAxes) {    
    // Left Joystick (Top/Bottom)
    float x = maxX*xAxes*2.0;

    // Left Joystick (Left/Right)
    float y = maxY*yAxes*2.0;
    
    // Right Joystick (Top/Bottom)
    lastInput.yaw =maxYaw*yawAxes*2.0;

    // Triggers
    // LT go down RT go up
    float z = 0.0;

    lastInput.axesInput = {x, y, z};
    lastInput.lastUpdate = ros::Time::now();
}
void ps4_remote::command_handle(const sensor_msgs::Joy::ConstPtr& msg) {
    if (sync) {
        // drones = mdp_api::get_all_rigidbodies();
        
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
        if (rtMax == 1.0 && ltMax == 1.0) {
            sync = true;
            ROS_INFO("Ready for take off");
        }
    }
   
}

void ps4_remote::control_update() {

    if (lastInput.axesInput[0] != 0.0f ||
        lastInput.axesInput[1] != 0.0f ||
        lastInput.axesInput[2] != 0.0f || 
        lastInput.yaw != 0.0f) {
        ROS_INFO("Control update: %d", lastInput.lastUpdate.nsec);
        highLevelCommand = false;
        if (!localCoord) {
            ROS_INFO("%s: Change position by [%.2f, %.2f, %.2f] and yaw by %f", drones[droneID].name.c_str(),
            lastInput.axesInput[0], lastInput.axesInput[1], lastInput.axesInput[2], lastInput.yaw);
            mdp::position_msg posMsg;
            posMsg.duration =  2.0f;
            posMsg.keepHeight = true;
            posMsg.relative = true;
            posMsg.position = lastInput.axesInput;
            posMsg.yaw = lastInput.yaw;
            mdp::set_drone_position(drones[droneID],posMsg);
        }
    }
    else {
        if (!highLevelCommand) {
            mdp::cmd_hover(drones[droneID]);
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
            maxRise = 0.5f;
            maxFall = 0.25f;
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
    auto drones = mdp::get_all_rigidbodies();
    drones = mdp::get_all_rigidbodies();
    ps4_remote::droneID = drones[0].numericID;
    auto followerDrone = drones[1];
    mdp::cmd_takeoff(followerDrone, 1.0f, 2.0f);
    mdp::sleep_until_idle(followerDrone);
    mdp::cmd_hover(drones[1]);

    if (drones.size() >= 2) {

        while (ros::ok() && !shouldFinish) {
            ros::spinOnce();

            if (sync) {
                control_update();
                auto currentPos = mdp::get_position(drones[0]);
                mdp::position_msg msg;
                msg.position = {currentPos.x, currentPos.y, 0.0f};
                msg.keepHeight = true;
                msg.relative = false;
                msg.duration = 1.0f;
                msg.yaw = 0.0f;
                mdp::set_drone_position(followerDrone, msg);
                loop_rate.sleep();
                ++count;
            }
        }

        mdp::go_to_home(drones[0], GO_TO_HOME);
        mdp::go_to_home(drones[1], GO_TO_HOME);
        mdp::sleep_until_idle(drones[0]);
        mdp::sleep_until_idle(drones[1]);
    }

}
void ps4_remote::terminate() {
    delete myNode;
    delete mySpin;
    ROS_INFO("Shutting Down Client API Connection");
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ps4_remote");
    ps4_remote::mySpin = new ros::AsyncSpinner(1,&ps4_remote::myQueue);
    mdp::initialise(SERVER_FREQ);
    bool start = true;

    ps4_remote::droneID = 0;
    ps4_remote::drones = mdp::get_all_rigidbodies();

    while (ps4_remote::drones[ps4_remote::droneID].name.find("object") != std::string::npos)
    {
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