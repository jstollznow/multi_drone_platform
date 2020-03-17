
/**
 * @defgroup public_api Public Api
 * @file user_api.h
 * @brief This is the header containing all user api functions used to create user programs capable of interacting
 * with the multi-drone platform
 * @ingroup public_api
 */

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <array>

/**
 * @brief The public facing namespace containing all user api functions used to create user programs capable of interacting
 * with the multi-drone platform
 */
namespace mdp {

/**
 * a structure representing a drone or rigidbody existing on the drone-server. It is not recommended to manually create
 * this structure and instead retrieve them from a call to get_all_rigidbodies()
 * @see get_all_rigidbodies
 */
struct id {
    uint32_t numericID;
    std::string name;
};

/**
 * a structure representing position information with a time stamp
 */
struct position_data {
    id respectiveID{};
    uint64_t timeStampNsec = 0; // fun-fact: valid until July 21, 2554
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float yaw = 0.0f;
    /**
     * checks whether the current structure data is valid. This will return false if the respective mdp::id does not
     * exist on the drone-server or if otherwise the containing data is invalid.
     * @return boolean whether the containing data is valid
     */
    bool isValid() const;
};

/**
 * a structure representing velocity information with a time stamp
 */
struct velocity_data {
    id respectiveID{};
    uint64_t timeStampNsec = 0; // fun-fact: valid until July 21, 2554
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float yawrate = 0.0f;
    /**
     * checks whether the current structure data is valid. This will return false if the respective mdp::id does not
     * exist on the drone-server or if otherwise the containing data is invalid.
     * @return boolean whether the containing data is valid
     */
    bool isValid() const;
};

/**
 * a structure used to input information to a call to set_drone_position()
 * @see set_drone_position
 */
struct position_msg {
    std::array<double, 3> position = {{0.0, 0.0, 0.0}};
    bool relative = false;
    bool keepHeight = false;
    double duration = 0.0;
    double yaw = 0.0;
};

/**
 * a structure used to input information to a call to set_drone_velocity()
 * @see set_drone_velocity
 */
struct velocity_msg {
    std::array<double, 3> velocity = {{0.0, 0.0, 0.0}};
    bool relative = false;
    bool keepHeight = false;
    double duration = 0.0;
    double yawRate = 0.0;
};

/**
 * a structure containing timings information returned by a call to get_operating_frequencies()
 * @see get_operating_frequencies
 */
struct timings {
    uint64_t timeStampNsec = 0;
    float moCapUpdateRate = 0.0f;
    float desDroneServerUpdateRate = 0.0f;
    float actualDroneServerUpdateRate = 0.0f;
    float timeToUpdateDrones = 0.0f;
    float waitTimePerFrame = 0.0f;

    /**
     * checks whether the current structure data is valid.
     * @return boolean
     */
    bool isValid() const;
};

/**
 * initialises the required data structures and connections to communicate with the multi-drone platform.
 * @param updateRate the desired update rate for this user application.
 */
void initialise(double updateRate);

/**
 * terminates the multi-drone platform application, de-allocating data structures created through initialise()
 * @see initialise
 */
void terminate();

/**
 * returns a list of all the rigidbodies currently active on the drone server.
 * @return a list of all active rigidbodies as mdp::id structures
 */
std::vector<mdp::id> get_all_rigidbodies();

/**
 * sets the desired velocity of the drone with the given mdp::id
 * @param id the id of the subject drone
 * @param msg a velocity message representing the desired velocity to set to
 */
void set_drone_velocity(const mdp::id& id, mdp::velocity_msg msg);

/**
 * returns the current velocity of the rigidbody with the given mdp::id
 * @param id the id of the subject rigidbody
 * @return a velocity_data structure containing the drones current velocity (does not update automatically)
 */
velocity_data get_velocity(const mdp::id& id);

/**
 * sets the desired position of the drone with the given mdp::id
 * @param id the id of the subject drone
 * @param msg a position message containing the desired position for the drone and timing information
 */
void set_drone_position(const mdp::id& id, mdp::position_msg msg);

/**
 * returns the current position of the rigidbody with the given mdp::id
 * @param id the id of the subject rigidbody
 * @return a position_data structure containing information on the drones current position
 */
position_data get_position(const mdp::id& id);

/**
 * sends a take off command to the drone with the given mdp::id. This takeoff movement will take duration seconds
 * to complete and will result in the drone hovering at the given height.
 * @param id the id of the subject drone
 * @param height the desired end height in meters of the takeoff command
 * @param duration the duration in seconds the takeoff will take
 */
void cmd_takeoff(const mdp::id& id, float height = 0.5f, float duration = 2.0f);

/**
 * send a land command to the drone with the given mdp::id
 * @param id the id of the subject drone
 * @param duration how long the drone should take to land in seconds
 */
void cmd_land(const mdp::id& id, float duration = 2.0f);

/**
 * sends an emergency command to the drone with the given id. This will prevent future messages being sent to this
 * drone and may cause the drone to fall out of flight. Only recommended to use as a last resort.
 * @param id the id of the subject drone
 */
void cmd_emergency(const mdp::id& id);

/**
 * sends a command to the drone with the given mdp::id to stop all motion and hover in place for the given duration.
 * @param id the id of the subject drone
 * @param duration the duration in seconds which the drone should hover for
 */
void cmd_hover(const mdp::id& id, float duration = 10.0f);

/**
 * sets the home location for the drone with the given mdp::id. This position is used on platform termination and
 * using the go_to_home(id) function. By default a drone's home position is set to its initial position.
 * @param id the id of the subject drone
 * @param msg a position_msg containing the new home position for the drone
 * @see go_to_home
 */
void set_home(const mdp::id& id, mdp::position_msg msg);

/**
 * gets the home position for the drone with the given mdp::id.
 * @param id the id of the subject drone
 * @return a position_data structure containing the drone's home position
 */
position_data get_home(const mdp::id& id);

/**
 * sends a command to the drone with the given mdp::id to return to it's home position.
 *
 * Setting the height parameter to a negative value will cause the drone to remain at its current height. Setting the
 * height parameter to 0.0 will cause the drone to automatically land upon reaching its home position. And setting
 * the height parameter to above 0.0 will cause the drone to return to it's home position at the given height in meters
 * @param id the id of the subject drone
 * @param duration the duration in seconds the drone should take to reach it's home position
 * @param height a value representing the desired end height of the drone (read description for details)
 */
void go_to_home(const mdp::id& id, float duration = 4.0f, float height = -1.0f);

/**
 * sets the update frequency for the drone server (default 100Hz)
 * @param updateFrequency the desired update frequency in Hertz
 */
void set_drone_server_update_frequency(float updateFrequency);

/**
 * returns a data structure containing various timing related information for the multi-drone platform
 * @return a timings data structure containing relevant information
 */
timings get_operating_frequencies();

/**
 * evokes the user system to update drone position and velocity information. will also quantize to the update rate
 * given in initialise(rate). It is recommended to call this function regularly throughout the program.
 * @see initialise
 */
void spin_once();

/**
 * halts the program until the drone with the given id returns a state of IDLE, LANDED, or DELETED. i.e. this function
 * only returns when the drone has finished it's last command.
 * @param id the id of the subject drone
 */
void sleep_until_idle(const mdp::id& id);

/**
 * gets the current state of the drone with the given mdp::id. states can be IDLE, LANDING, LANDED, MOVING, TAKING_OFF,
 * or DELETED
 * @param id the id of the subject drone
 * @return a string representing the drones current state
 */
std::string get_state(const mdp::id& id);

}
