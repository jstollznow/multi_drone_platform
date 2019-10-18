#pragma once

#include <stdint.h>
#include <string>
#include <vector>
#include <array>

namespace mdp_api {

    struct id {
        uint32_t numeric_id;
        std::string name;
    };

    struct position_data {
        public:
            float x, y, z;
            float yaw;
    };

    struct position_msg {
        private:
            int target_id = -1;
        public:
            std::array<double, 3> position = {{0.0, 0.0, 0.0}};
            std::array<bool, 3> relative = {{false,false,false}};
            double duration = 0.0;
            double yaw = 0.0;

            void set_as_relative(bool pValue);
            void set_target(mdp_api::id pTarget);
            void rem_target();
    };

    struct velocity_msg {
        public:
            std::array<double, 3> velocity = {{0.0, 0.0, 0.0}};
            std::array<bool, 3> relative = {{false,false,false}};
            double duration = 0.0;
            double yaw_rate = 0.0;

            void set_as_relative(bool pValue);
    };

    struct timings {
        float motion_capture_update_rate;
        float desired_drone_server_update_rate;
        float achieved_drone_server_update_rate;
        float time_to_update_drones;
        float wait_time_per_frame;
    };

    typedef position_data velocity_data;

    void initialise(unsigned int pUpdateRate);
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

    void set_drone_server_update_frequency(float pUpdateFrequency);
    timings get_operating_frequencies();

    void spin_once();
}