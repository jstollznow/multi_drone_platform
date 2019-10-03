#include <stdint.h>
#include <vector>

#define FRAME_ID "user_api"
#define PUB_TOPIC "mdp_api"
#define FEEDBACK_TOPIC "mdp_api_srv"
#define LOOP_RATE_HZ 100

namespace mdp_api {

    struct position_data {
        uint32_t droneID;
        float x, y, z;
        float yaw;
    };
    typedef position_data velocity_data;

    void initialise(int argc, char **argv);
    void terminate();

    std::vector<uint32_t> get_all_rigidbodies();
    void set_drone_velocity(uint32_t pDroneID, float pVelX, float pVelY, float pVelZ, float pYawRate = 0.0f);
    void set_drone_position(uint32_t pDroneID, float pPosX, float pPosY, float pPosZ, float pDuration = 0.0f, float pYaw = 0.0f);
    position_data get_body_position(uint32_t pRigidbodyID);
    velocity_data get_body_velocity(uint32_t pRigidbodyID);
}