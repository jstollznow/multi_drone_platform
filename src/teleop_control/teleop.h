//
// Created by jacob on 18/3/20.
//
#include "ros/ros.h"
#include "user_api.h"
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>
#include <ros/callback_queue.h>

#include "../debug/logger/logger.h"

#include <sstream>
#include <queue>

#ifndef MULTI_DRONE_PLATFORM_TELEOP_H
#define MULTI_DRONE_PLATFORM_TELEOP_H

class teleop {
public:
    void run();
    explicit teleop(std::vector<mdp::id>& drones);
    void log (logger::log_type logLevel, const std::string& msg);
    void terminate();
private:
    enum commandType {
        TAKEOFF,
        LAND,
        HOVER,
        GO_TO_HOME,
        ID_SWITCH_UP,
        ID_SWITCH_DOWN
    };

    struct {
        std::array<double, 3> axesInput;
        float yaw;
    } teleopInput;

    ros::NodeHandle node;
    ros::AsyncSpinner spin;
    ros::CallbackQueue queue;

    mdp::velocity_data currentVelocity;
    std::vector<mdp::id> drones;
    int controlIndex;
    std::queue<commandType> commandQueue;
    ros::Time timeoutEnd;

    bool emergency;
    bool isSynced;

    float maxX;
    float maxY;
    float maxRise;
    float maxFall;
    float maxYaw;

    float ltMax;
    float rtMax;

    int allEmergencyInput;
    int selectedEmergencyInput;
    int changeDroneInput;
    int safeShutdownInput;
    int takeoffInput;
    int landInput;
    int hoverInput;
    int goToHomeInput;
    float xAxesInput;
    float yAxesInput;
    float incAltitudeInput;
    float decAltitudeInput;
    float yawAxes;

    ros::Publisher emergencyPublisher;
    ros::Subscriber joySubscriber;
    ros::Publisher logPublisher;

    void set_limits();
    void input_callback(const sensor_msgs::Joy::ConstPtr& msg);
    void control_update();

    void command_handle();
    bool emergency_handle();
    bool option_change_handle();
    bool high_lvl_command_handle();
    bool last_input_handle();

    std::array<double, 3> input_capped(std::array<double, 3> requestedVelocity);
    double yaw_capped();
    void joystick_command(std::array<double, 3> vel);
    bool stable_for_command();
    void id_switch(bool up);
    void clear_command_queue();

};


#endif //MULTI_DRONE_PLATFORM_TELEOP_H
