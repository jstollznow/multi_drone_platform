//
// Created by jacob on 21/5/20.
//
#include "user_api.h"
int main() {
    mdp::initialise(10, "dummy_variables");
    auto drones = mdp::get_all_rigidbodies();
    if (drones.size() > 1) {
        mdp::cmd_takeoff(drones[0], 20.0, 0.1);
        mdp::sleep_until_idle(drones[0]);
        mdp::position_msg posMsg;
        posMsg.duration = 0.1f;
        posMsg.position = {10.0, -10.0, -19.0};
        posMsg.relative = true;
        posMsg.keepHeight = true;
        mdp::set_drone_position(drones[0], posMsg);
        mdp::sleep_until_idle(drones[0]);
        mdp::cmd_land(drones[0], -1.0);
        mdp::sleep_until_idle(drones[0]);

    }
    mdp::terminate();
    return 0;
}
