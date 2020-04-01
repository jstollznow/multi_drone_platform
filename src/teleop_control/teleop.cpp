#include "teleop.h"

#define NODE_NAME "teleop"
#define INPUT_TOPIC "/ps4"
#define UPDATE_RATE 10

#define TAKEOFF_TIME 3.0f
#define LAND_TIME 3.0f
#define HOVER_TIME 10.0f
#define GO_TO_HOME_TIME 4.0f

#define MSG_DUR 2.0f
// LIMIT LEVEL
// 0 DEMO - SAFE
// 1 MORE CONTROL
#define LIMIT_LEVEL 0

teleop::teleop(std::vector<mdp::id> drones):spin(1, &queue) {
    this->drones = drones;
    this->controlIndex = 0;
    this->highLevelCommand = false;
    this->emergency = false;
    set_limits();

    node = ros::NodeHandle();
    node.setCallbackQueue(&queue);

    std::string logTopic = "/" + (std::string)NODE_NAME + "/log";

    logPublisher = node.advertise<multi_drone_platform::log> (logTopic, 20);
    joySubscriber = node.subscribe<sensor_msgs::Joy>(INPUT_TOPIC, 1, &teleop::input_callback, this);

    this->log(logger::INFO, "Subscribing to " + std::string(INPUT_TOPIC));
}

void teleop::run() {
    ros::Rate loop_rate(UPDATE_RATE);
    this->log(logger::INFO, "Initialised PS4 Remote");

    sync = false;

    ltMax = 0.0f;
    rtMax = 0.0f;

    set_limits();

    this->log(logger::INFO, "Please sync remote by pressing the left and right triggers...");

    spin.start();

    while (ros::ok() && !emergency) {
        ros::spinOnce();
        if (sync) {
            control_update();
            loop_rate.sleep();
        }
    }
}

void teleop::log(logger::log_type logLevel, std::string msg) {
    logger::post_log(logLevel, NODE_NAME, msg, logPublisher);
}

bool teleop::emergency_handle(int allDronesButton, int oneDroneButton) {
    // PS Button
    if (allDronesButton) {
        emergency = true;
        this->log(logger::WARN, "ALL EMERGENCY requested");
        for(size_t i = 0; i < drones.size(); i++)
        {
            mdp::cmd_emergency(drones[i]);
        }
        return true;
    }
    // Share Button
    else if (oneDroneButton) {
        if (drones.size() > controlIndex) {
            emergency = true;
            this->log(logger::WARN, "EMERGENCY requested for " + drones[controlIndex].name);
            mdp::cmd_emergency(drones[controlIndex]);
        }
        return true;
    }
    return false;
}
bool teleop::option_change_handle(float idChange, int coordChange) {

    // up or down D-Pad
    if ((int)idChange != 0) {
        highLevelCommand = true;
        this->log(logger::DEBUG, "ID Change requested");
        this->log(logger::DEBUG, "Changing from " + drones[controlIndex].name);
        mdp::cmd_hover(drones[controlIndex]);

        // will go up or down according to input
        controlIndex += (int)idChange;

        if (controlIndex < 0) controlIndex = drones.size() - 1;
        if (controlIndex >= drones.size()) controlIndex = 0;
        this->log(logger::DEBUG, "Now controlling " + drones[controlIndex].name);

        return true;
    }
//    // options button
//    else if (coordChange != 0) {
//        highLevelCommand = true;
//        if (localCoord) {
//            localCoord = false;
//            ROS_INFO("%s: Changing control to absolute coord", drones[controlIndex].name.c_str());
//        }
//        else {
//            localCoord = true;
//            ROS_INFO("%s: Changing control to relative coord", drones[controlIndex].name.c_str());
//        }
//        return true;
//    }
    return false;
}

bool teleop::high_lvl_command_handle(int takeoff, int land, int hover, int goToHome) {

    // cross
    if (takeoff) {
        highLevelCommand = true;
        highLevelCommandEnd = ros::Time::now() + ros::Duration(TAKEOFF_TIME);
        this->log(logger::DEBUG, "Takeoff requested for " + drones[controlIndex].name);
        mdp::cmd_takeoff(drones[controlIndex], 0.5f, TAKEOFF_TIME);
        return true;
    }
        // circle
    else if (land) {
        highLevelCommand = true;
        highLevelCommandEnd = ros::Time::now() + ros::Duration(LAND_TIME);
        this->log(logger::DEBUG, "Land requested for " + drones[controlIndex].name);
        mdp::cmd_land(drones[controlIndex], LAND_TIME);
        return true;
    }
        // triangle
    else if (hover) {
        highLevelCommand = true;
        highLevelCommandEnd = ros::Time::now() + ros::Duration(HOVER_TIME);
        this->log(logger::DEBUG, "Hover requested for " + drones[controlIndex].name);
        mdp::cmd_hover(drones[controlIndex], HOVER_TIME);
        return true;
    }
        // square
    else if (goToHome) {
        highLevelCommand = true;
        highLevelCommandEnd = ros::Time::now() + ros::Duration(GO_TO_HOME_TIME + 3.0f);
        this->log(logger::DEBUG, "GoToHome requested for " + drones[controlIndex].name);
        mdp::go_to_home(drones[controlIndex], GO_TO_HOME_TIME);
        return true;
    }

    return false;
}

bool teleop::last_input_handle(float xAxes, float yAxes, float zUpTrigger, float zDownTrigger, float yawAxes) {
    // Left Joystick (Top/Bottom)
    float x = maxX * xAxes * MSG_DUR;

    // Left Joystick (Left/Right)
    float y = maxY * yAxes * MSG_DUR;

    // Right Joystick (Top/Bottom)
    teleopInput.yaw = maxYaw * yawAxes * MSG_DUR;

    // Triggers
    // LT go down RT go up
    float z = ((maxFall) / MSG_DUR) * (zDownTrigger - 1) - ((maxRise) / MSG_DUR) * (zUpTrigger - 1);

    teleopInput.axesInput = {x, y, z};
    teleopInput.lastUpdate = ros::Time::now();
}
void teleop::command_handle(const sensor_msgs::Joy::ConstPtr& msg) {

    if (sync) {
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
            this->log(logger::INFO, "Ready for take off");
        }
    }

}

std::array<double, 3> teleop::input_capped() {
    double maxVelChange = 1.0;
    std::array<double, 3> ret;
    for (int i = 0; i < 3; i ++) {
        double dir = (teleopInput.axesInput[i]/std::abs(teleopInput.axesInput[i]));
        double relChange = std::min(maxVelChange, std::abs(lastMsgSent.velocity[i] - teleopInput.axesInput[i]));
        ret[i] = dir * relChange;

    }
    return ret;
}

double teleop::yaw_capped() {

}

void teleop::control_update() {

    if (teleopInput.axesInput[0] != 0.0f || teleopInput.axesInput[1] != 0.0f || teleopInput.axesInput[2] != 0.0f || teleopInput.yaw != 0.0f) {
        highLevelCommand = false;
    }

    if (!highLevelCommand) {
        ROS_INFO("%s: Change position by [%.2f, %.2f, %.2f] and yaw by %f", drones[controlIndex].name.c_str(),
                 teleopInput.axesInput[0], teleopInput.axesInput[1], teleopInput.axesInput[2], teleopInput.yaw);
        mdp::velocity_msg velocityMsg;
        velocityMsg.duration = MSG_DUR;
        velocityMsg.keepHeight = true;
        velocityMsg.relative = true;
        velocityMsg.velocity = input_capped();
        velocityMsg.yawRate = teleopInput.yaw;
        lastMsgSent = velocityMsg;
        mdp::set_drone_velocity(drones[controlIndex], velocityMsg);
    }
    else if (ros::Time::now().toNSec() > highLevelCommandEnd.toNSec()){
        mdp::cmd_hover(drones[controlIndex], HOVER_TIME);
        highLevelCommandEnd = ros::Time::now() + ros::Duration(HOVER_TIME);
        highLevelCommand = true;
    }
}

void teleop::input_callback(const sensor_msgs::Joy::ConstPtr& msg) {
    command_handle(msg);
}
void teleop::set_limits() {
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

void teleop::terminate() {
    this->log(logger::INFO, "Shutting Down PS4 Teleop");
}