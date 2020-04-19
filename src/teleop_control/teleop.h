//
// Created by jacob on 18/3/20.
//
#include "ros/ros.h"
#include "user_api.h"
#include "sensor_msgs/Joy.h"
#include "../debug/logger/logger.h"
#include <ros/callback_queue.h>

#include <sstream>
#include <queue>

#ifndef MULTI_DRONE_PLATFORM_TELEOP_H
#define MULTI_DRONE_PLATFORM_TELEOP_H

struct input{
    ros::Time lastUpdate;
    std::array<double, 3> axesInput;
    float yaw;
};

enum command {
    TAKEOFF, LAND, HOVER, GO_TO_HOME
};

class teleop {
public:
    void run();
    ros::NodeHandle node;
    ros::AsyncSpinner spin;
    ros::CallbackQueue queue;
    explicit teleop(std::vector<mdp::id> drones);
    void log (logger::log_type logLevel, std::string msg);
private:
    input teleopInput;
    mdp::velocity_msg lastMsgSent;
    std::vector<mdp::id> drones;
    int controlIndex;
    std::queue<command> commandQueue;
    bool highLevelCommand;
    ros::Time highLevelCommandEnd;
    bool emergency;
    float maxYaw;
    float maxX;
    float maxY;
    float maxRise;
    float maxFall;

    bool sync;
    float ltMax;
    float rtMax;

    ros::Subscriber joySubscriber;
    ros::Publisher logPublisher;

    void set_limits();
    void input_callback(const sensor_msgs::Joy::ConstPtr& msg);
    void control_update();


    void command_handle(const sensor_msgs::Joy::ConstPtr& msg);
    bool emergency_handle(int allDronesButton, int oneDroneButton);
    bool option_change_handle(float idChange, int coordChange);
    bool high_lvl_command_handle(int takeoff, int land, int hover, int goToHome);
    bool last_input_handle(float xAxes, float yAxes, float zUpTrigger, float zDownTrigger, float yawAxes);
    void terminate();

    std::array<double, 3> input_capped();
    double yaw_capped();
    void joystick_command(std::array<double, 3> vel);

};


#endif //MULTI_DRONE_PLATFORM_TELEOP_H
