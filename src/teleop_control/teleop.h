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

/**
 * ROS topics referenced throughout
 */
#define NODE_NAME "teleop"
#define INPUT_TOPIC "/controller"
#define EMERGENCY_TOPIC "mdp_emergency"

/**
 * Operating frequency of the ROS control loop
 */
#define UPDATE_RATE 10

/**
 * Duration applied for each command type
 */
#define TAKEOFF_TIME 3.0f
#define LAND_TIME 3.0f
#define HOVER_TIME 10.0f
#define GO_TO_HOME_TIME 4.0f

/**
 * Velocity in any direction deemed greater than this constant breaks the stationary boolean (see stable_for_command)
 */
#define THRESHOLD_VEL 0.01f

/**
 * Message duration applied to velocity joystick commands
 */
#define MSG_DUR 2.0f

/**
 * Limit level determines how controllable the drone is, higher limits translate to more control, but also results in
 * more instability. 0 - Demo (safe), 1 - Developer (less safe)
 */
#define LIMIT_LEVEL 0

/**
 * Differences in axes array size in the joy message determine which remote is being used.
 */
#define PS4 8
#define PS3 6

class teleop {
public:
    /**
     * The run method iterates ros to ensure remote controller commands are processed at a rate determined by the
     * constant UPDATE_RATE.
     */
    void run();

    /**
     * This is the constructor method and should be declared with a vector of drone ids of objects on the drone server.
     * This method establishes the necessary topics and parameters to operate the teleop program.
     * @param drones The vector of drones that are being tracked and are on the controllable through the drone server.
     */
    explicit teleop(std::vector<mdp::id>& drones);

    /**
     * Log is a shortcut method used to call the global logging procedure for the teleop program.
     * @param logLevel Defines the verbosity level for the log message.
     * @param msg The message to be logged.
     */
    void log (logger::log_type logLevel, const std::string& msg);
    /**
     * Called when the controller program is shutdown.
     */
    void terminate();
private:
    /**
     * To differentiate the commands in internal queuing processes, an enum type was used.
     */
    enum commandType {
        TAKEOFF,
        LAND,
        HOVER,
        GO_TO_HOME,
        ID_SWITCH_UP,
        ID_SWITCH_DOWN
    };

    /**
     * Used to track the most recent controller input and then used to apply an appropriate drone command.
     */
    struct {
        std::array<double, 3> axesInput;
        float yaw;
    } teleopInput;

    /**
     * Various ROS types to use asynchronous ROS events.
     */
    ros::NodeHandle node;
    ros::AsyncSpinner spin;
    ros::CallbackQueue queue;

    /**
     * list of controllable drones
     */
    std::vector<mdp::id> drones;

    /**
     * The given drone's current velocity
     */
    mdp::velocity_data currentVelocity;


    /**
     * Determines which drone the controller is currently controlling
     */
    int controlIndex;

    /**
     * Command queue to manage various command sequences, such as calling a hover, after the drone has come to a stop.
     */
    std::queue<commandType> commandQueue;

    /**
     * Timeout used to trigger hover command if nothing is required from the drone
     */
    ros::Time timeoutEnd;

    /**
     * Used directly in ROS cycle, emergency can be called using the remote and stop ROS iterations.
     */
    bool emergency;

    /**
     * Ensures command cannot be issued until the remote has been 'synced'. Used to ensure left and right triggers have
     * been pressed in all the way.
     */
    bool isSynced;
    float ltMax;
    float rtMax;

    /**
     * Used for limits dependent on the compile time mode set.
     */
    float maxX;
    float maxY;
    float maxRise;
    float maxFall;
    float maxYaw;


    /**
     * Used to capture the user input for each post to the relevant controller input topic as determined by the joy
     * package.
     */
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

    /**
     * Used to publish emergency commands when necessary
     */
    ros::Publisher emergencyPublisher;

    /**
     * Listens to remote control input asynchronously, reacting immediately to high level commands and periodically to
     * joystick commands.
     */
    ros::Subscriber joySubscriber;

    /**
     * Used to manage log messages.
     */
    ros::Publisher logPublisher;

    /**
     * Sets controller limits depending on compile-time-set mode (LIMIT_LEVEL), called from the constructor.
     */
    void set_limits();

    /**
     * Callback method for controller input topic
     * @param msg A joy message input containing the input from the remote controller. The format of the message
     * determines the type of remote being used.
     */
    void input_callback(const sensor_msgs::Joy::ConstPtr& msg);

    /**
     * Main control method of the teleop program. Determines which command to apply to the selected drone (determined
     * by control index). The order of priority is joystick commands, followed by high level commands, then finally if
     * joystick input is non existent and no high level commands have been queued, a hover command is applied.
     */
    void control_update();

    /**
     * Called each time a new controller input is published. This controls the order in which specific command handles
     * are called. Emergency has the highest priority, followed by option changes, high level commands, then finally
     * joystick input.
     */
    void command_handle();

    /**
     * Manages the two emergency inputs. All emergency has priority over selected emergency.
     * @return Returns true if one of the two buttons has been clicked.
     */
    bool emergency_handle();

    /**
     * Manages the ability to change the drone which is being controlled. Manages the vector of drones as a cyclic list,
     * in which case, if the controlIndex is 0 and down on the D-Pad is clicked, the next controlIndex will be the end
     * of the vector.
     * @return Will return true if an id change has been requested.
     */
    bool option_change_handle();

    /**
     * Manages the high level commands for the selected drone including takeoff, land, hover and goToHome.
     * @return Returns true if either of these four commands have been requested.
     */
    bool high_lvl_command_handle();

    /**
     * This method changes the joystick inputs, using them with the limits to determine the next periodic joystick
     * command. This will be overridden each time a new input is published, the periodic nature of the joystick commands
     * means only the last input matters.
     */
    void last_input_handle();

    /**
     * This method ensures that the drones velocity does not change too rapidly between periodic joystick commands.
     * If the requested command is too large of a change, the input will be capped to ensure the drone's movement is
     * smooth.
     * @param requestedVelocity The requested velocity is determined by the joystick input.
     * @return This method will return the requested velocity if it is within the maximum allowable change, it will
     * return a capped velocity if it is not.
     */
    std::array<double, 3> input_capped(std::array<double, 3> requestedVelocity);

    /**
     * Similar to input capped this will check the current yaw and the requested yaw to determine if this change is
     * appropriate.
     * @return Will return the next desired yaw, which will be capped according to the maximum yaw change.
     */
    double yaw_capped(float requestedYaw);

    /**
     * This method publishes a velocity command to the relevant drone topic, using the input_capped method to determine
     * the desired velocity.
     * @param vel The input velocity is used as a parameter for the input_capped method to determine the next requested
     * velocity.
     */
    void joystick_command(std::array<double, 3> vel);

    /**
     * For specific commands such as land and hover, the drone should be almost stationary in the air to apply the
     * given command. This bool returns a bool which reflects whether the drone's velocity is approximately zero.
     * @return Returns whether the drone is approximately stationary.
     */
    bool stable_for_command();

    /**
     * This method issues a hover command to the drone that was being controlled and then increments or decrements
     * controlIndex to determine the next controllable drone. The controllable drones vector is managed in a circular
     * fashion to ensure all drones can easily be cycled through.
     * @param up Indicates whether to increase or decrease controlIndex.
     */
    void id_switch(bool up);

    /**
     * Resets the command queue used for high level commands, used when switching between drones.
     */
    void clear_command_queue();
};


#endif //MULTI_DRONE_PLATFORM_TELEOP_H
