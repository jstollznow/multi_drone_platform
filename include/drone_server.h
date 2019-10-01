#include <stdint.h>
#include <vector>

#include "../src/objects/rigidBody.h"

#define FRAME_ID "drone_server"
#define PUB_TOPIC "mdp_server"
#define FEEDBACK_TOPIC "mdp_server_srv"
#define LOOP_RATE_HZ 100

namespace mdp_server {

    std::vector<rigidBody> drones;
    
}
