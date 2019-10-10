#pragma once

#include <stdint.h>
#include <string>
#include <vector>

#define FRAME_ID "user_api"
#define PUB_TOPIC "mdp_api"
#define FEEDBACK_TOPIC "mdp_api_srv"
#define LOOP_RATE_HZ 100

namespace mdp_api {

    struct position_data {
        float x, y, z;
        float yaw;
    };

    struct id {
        uint32_t numeric_id;
        std::string name;
    };

    typedef position_data velocity_data;

    void initialise();
    void terminate();

    std::vector<mdp_api::id> get_all_rigidbodies();

    void set_drone_velocity(mdp_api::id pDroneID, float pVelX, float pVelY, float pVelZ, float pYawRate = 0.0f);
    velocity_data get_body_velocity(mdp_api::id pRigidbodyID);

    void set_drone_position(mdp_api::id pDroneID, float pPosX, float pPosY, float pPosZ, float pDuration = 0.0f, float pYaw = 0.0f);
    position_data get_body_position(mdp_api::id pRigidbodyID);

    void cmd_takeoff(mdp_api::id pDroneID);
    void cmd_land(mdp_api::id pDroneID);
    void cmd_emergency(mdp_api::id pDroneID);
    void cmd_hover(mdp_api::id pDroneID);

    void set_home(mdp_api::id pDroneID, float pPosX, float pPosY, float pPosZ);
    position_data get_home(mdp_api::id pDroneID);
    void goto_home(mdp_api::id pDroneID);
}