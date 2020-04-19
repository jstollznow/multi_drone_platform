#include "teleop.h"

#define NODE_NAME "teleop"
#define INPUT_TOPIC "/ps4"
#define EMERGENCY_TOPIC "mdp_emergency"
#define UPDATE_RATE 10

#define TAKEOFF_TIME 3.0f
#define LAND_TIME 3.0f
#define HOVER_TIME 10.0f
#define GO_TO_HOME_TIME 4.0f
#define THRESHOLD_VEL 0.01f

#define MSG_DUR 2.0f
// LIMIT LEVEL
// 0 DEMO - SAFE
// 1 MORE CONTROL
#define LIMIT_LEVEL 0



teleop::teleop(std::vector<mdp::id> drones):spin(1, &queue) {
    this->drones = drones;
    this->controlIndex = 0;
    this->emergency = false;

    node = ros::NodeHandle();

    node.setCallbackQueue(&queue);

    std::string logTopic = "/mdp/" + (std::string)NODE_NAME + "/log";

    logPublisher = node.advertise<multi_drone_platform::log> (logTopic, 20);
    joySubscriber = node.subscribe<sensor_msgs::Joy>(INPUT_TOPIC, 1, &teleop::input_callback, this);
    emergencyPublisher = node.advertise<std_msgs::Empty>(EMERGENCY_TOPIC, 1);

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
    logger::post_log(logLevel, NODE_NAME, logPublisher, msg);
}

bool teleop::emergency_handle(int allDronesButton, int oneDroneButton, int safeShutdownButton) {
    // PS Button
    if (allDronesButton) {
        emergency = true;
        this->log(logger::WARN, "ALL EMERGENCY requested");
        emergencyPublisher.publish(std_msgs::Empty());
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
    // Options Button
    else if (safeShutdownButton) {
        emergency = true;
        node.setParam("mdp/should_shut_down", true);
        this->log(logger::WARN, "Safe server shutdown requested");
        return true;
    }
    return false;
}
bool teleop::option_change_handle(float idChange) {

    // up or down D-Pad
    if ((int)idChange != 0) {
        clear_command_queue();

        this->log(logger::DEBUG, "ID Change requested");
        this->log(logger::DEBUG, "Changing from " + drones[controlIndex].name);
        if(idChange > 0) {
            commandQueue.push(ID_SWITCH_UP);
        }
        else {
            commandQueue.push(ID_SWITCH_DOWN);
        }

        return true;
    }
    return false;
}

bool teleop::high_lvl_command_handle(int takeoff, int land, int hover, int goToHome) {

    // cross
    if (takeoff) {
        clear_command_queue();
        this->log(logger::DEBUG, "Takeoff requested for " + drones[controlIndex].name);
        commandQueue.push(TAKEOFF);
        return true;
    }
    // circle
    else if (land) {
        clear_command_queue();
        this->log(logger::DEBUG, "Land requested for " + drones[controlIndex].name);
        commandQueue.push(LAND);
        return true;
    }
    // triangle
    else if (hover) {
        clear_command_queue();
        this->log(logger::DEBUG, "Hover requested for " + drones[controlIndex].name);
        commandQueue.push(HOVER);
        return true;
    }
    // square
    else if (goToHome) {
        clear_command_queue();
        this->log(logger::DEBUG, "GoToHome requested for " + drones[controlIndex].name);
        commandQueue.push(GO_TO_HOME);
        return true;
    }

    return false;
}

bool teleop::last_input_handle(float xAxes, float yAxes, float zUpTrigger, float zDownTrigger, float yawAxes) {

    // Left Joystick (Top/Bottom)
    float x = maxX * xAxes * MSG_DUR;

    // Left Joystick (Left/Right)
    float y = maxY * yAxes * MSG_DUR;

    // Triggers
    // LT go down RT go up
    float z = ((maxFall) / MSG_DUR) * (zDownTrigger - 1) - ((maxRise) / MSG_DUR) * (zUpTrigger - 1);

    // Right Joystick (Top/Bottom)
    teleopInput.yaw = maxYaw * yawAxes * MSG_DUR;

    teleopInput.axesInput = {x, y, z};
}
void teleop::command_handle(const sensor_msgs::Joy::ConstPtr& msg) {

    if (sync) {
        if (!emergency_handle(msg->buttons[10], msg->buttons[8], msg->buttons[9])) {
            if (!option_change_handle(msg->axes[7])) {
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

std::array<double, 3> teleop::input_capped(std::array<double, 3> requestedVelocity) {
    double maxVelChange = 0.25;
    std::array<double, 3> ret;
    std::array<double, 3> currVelArr = {currentVelocity.x, currentVelocity.y, currentVelocity.z};
    for (int i = 0; i < 3; i ++) {
        double dir = (teleopInput.axesInput[i] - currVelArr[i])/std::abs(teleopInput.axesInput[i] - currVelArr[i]);
        double relChange = std::min(maxVelChange, std::abs(teleopInput.axesInput[i] - currVelArr[i]));
        if (std::isnan(dir)) dir = 1.0f;

        ret[i] =  currVelArr[i] + dir * relChange;
//        if (ret[i] != 0) {
//            this->log(logger::DEBUG,
//                    "teleop: " +
//                    std::to_string(teleopInput.axesInput[i]) +
//                    " lastMsg: " +
//                    std::to_string(currVelArr[i]) +
//                    " Rel: " + std::to_string(ret[i]));
//        }
    }
    return ret;
}

double teleop::yaw_capped() {

}

void teleop::joystick_command(std::array<double, 3> vel) {
    mdp::velocity_msg velocityMsg;
    velocityMsg.duration = MSG_DUR;
    velocityMsg.keepHeight = true;
    velocityMsg.relative = true;
    velocityMsg.velocity = input_capped(vel);
    velocityMsg.yawRate = teleopInput.yaw;

    mdp::set_drone_velocity(drones[controlIndex], velocityMsg);

    // ready for joystick_input whenever
    timeoutEnd = ros::Time::now();
}

bool teleop::stable_for_command() {
    return (std::abs(currentVelocity.x) <= THRESHOLD_VEL &&
            std::abs(currentVelocity.y) <= THRESHOLD_VEL &&
            std::abs(currentVelocity.z) <= THRESHOLD_VEL);
}

void teleop::id_switch(bool up) {
    controlIndex += up ? 1 : -1;
    if (controlIndex < 0) controlIndex = drones.size() - 1;
    if (controlIndex >= drones.size()) controlIndex = 0;
    this->log(logger::DEBUG, "Now controlling " + drones[controlIndex].name);
}

void teleop::clear_command_queue() {
    commandQueue = std::queue<command>();
}

void teleop::control_update() {
    bool joystickInput = false;
    currentVelocity = mdp::get_velocity(drones[controlIndex]);

    if (teleopInput.axesInput[0] != 0.0f || teleopInput.axesInput[1] != 0.0f || teleopInput.axesInput[2] != 0.0f || teleopInput.yaw != 0.0f) {
        joystickInput = true;
    }

    if (joystickInput) {
        joystick_command(teleopInput.axesInput);
    }
    else if (!commandQueue.empty()){
        auto front = commandQueue.front();
        timeoutEnd = ros::Time::now();
        switch(front) {
            case GO_TO_HOME:
                mdp::go_to_home(drones[controlIndex], GO_TO_HOME_TIME);

                // to allow for move then land, added 3 seconds for move
                timeoutEnd += ros::Duration(GO_TO_HOME_TIME + 3.0f);
                commandQueue.pop();
                break;

            case TAKEOFF:
                mdp::cmd_takeoff(drones[controlIndex], 0.5f, TAKEOFF_TIME);
                timeoutEnd += ros::Duration(TAKEOFF_TIME);
                commandQueue.pop();
                break;

            case LAND:
                if (stable_for_command()) {
                    this->log(logger::DEBUG, "Executing land");
                    mdp::cmd_land(drones[controlIndex], LAND_TIME);
                    timeoutEnd += ros::Duration(LAND_TIME);
                    commandQueue.pop();
                }
                else {
                    joystick_command(std::array<double,3>{0.0f, 0.0f, 0.0f});
                }
                break;

            case HOVER:
                if (stable_for_command()) {
                    this->log(logger::DEBUG, "Executing hover");
                    mdp::cmd_hover(drones[controlIndex], HOVER_TIME);
                    timeoutEnd += ros::Duration(HOVER_TIME);
                    commandQueue.pop();
                }
                else {
                    joystick_command(std::array<double,3>{0.0f, 0.0f, 0.0f});
                }
                break;

            case ID_SWITCH_UP:
                if (stable_for_command()) {
                    this->log(logger::DEBUG, "Executing ID switch UP");
                    mdp::cmd_hover(drones[controlIndex], HOVER_TIME);
                    timeoutEnd += ros::Duration(HOVER_TIME);
                    id_switch(true);
                    commandQueue.pop();
                }
                else {
                    joystick_command(std::array<double,3>{0.0f, 0.0f, 0.0f});
                }
                break;

            case ID_SWITCH_DOWN:
                if (stable_for_command()) {
                    this->log(logger::DEBUG, "Executing ID switch DOWN");
                    mdp::cmd_hover(drones[controlIndex], HOVER_TIME);
                    timeoutEnd += ros::Duration(HOVER_TIME);
                    id_switch(false);
                    commandQueue.pop();
                }
                else {
                    joystick_command(std::array<double,3>{0.0f, 0.0f, 0.0f});
                }
                break;
        }
    }
    else if (ros::Time::now().toNSec() > timeoutEnd.toNSec()) {
        clear_command_queue();
        commandQueue.push(HOVER);
        this->log(logger::DEBUG, "Queuing hover due to no input");
    }
}

void teleop::input_callback(const sensor_msgs::Joy::ConstPtr& msg) {
    command_handle(msg);
}
//@TODO: configure limit modes
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