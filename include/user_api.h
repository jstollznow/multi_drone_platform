#pragma once

#include <stdint.h>
#include <string>
#include <vector>
#include <array>
// #include <boost/python.hpp>



namespace mdp_api {

struct id {
    uint32_t numericID;
    std::string name;
};

struct position_data {
    uint64_t timeStampNsec; // fun-fact: valid until July 21, 2554
    float x, y, z;
    float yaw;
};

struct position_msg {
    std::array<double, 3> position = {{0.0, 0.0, 0.0}};
    bool relative = false;
    bool keepHeight = false;
    double duration = 0.0;
    double yaw = 0.0;
};

struct velocity_msg {
    std::array<double, 3> velocity = {{0.0, 0.0, 0.0}};
    bool relative = false;
    bool keepHeight = false;
    double duration = 0.0;
    double yawRate = 0.0;
};

struct timings {
    float moCapUpdateRate;
    float desDroneServerUpdateRate;
    float actualDroneServerUpdateRate;
    float timeToUpdateDrones;
    float waitTimePerFrame;
};

typedef position_data velocity_data;

void initialise(unsigned int pUpdateRate);
void terminate();

std::vector<mdp_api::id> get_all_rigidbodies();

void set_drone_velocity(mdp_api::id pDroneID, mdp_api::velocity_msg pMsg);
velocity_data get_velocity(mdp_api::id pRigidbodyID);

void set_drone_position(mdp_api::id pDroneID, mdp_api::position_msg pMsg);
position_data get_position(mdp_api::id pRigidbodyID);

void cmd_takeoff(mdp_api::id pDroneID, float pHeight = 0.5f, float pDuration = 2.0f);
void cmd_land(mdp_api::id pDroneID);
void cmd_emergency(mdp_api::id pDroneID);
void cmd_hover(mdp_api::id pDroneID);

void set_home(mdp_api::id pDroneID, mdp_api::position_msg pMsg);
position_data get_home(mdp_api::id pDroneID);
void go_to_home(mdp_api::id pDroneID, float duration = 4.0f, float pHeight = -1.0f);

void set_drone_server_update_frequency(float pUpdateFrequency);
timings get_operating_frequencies();

void spin_once();
int rate();
void sleep_until_idle(mdp_api::id pDroneID);
std::string get_state(mdp_api::id pDroneID);

}

