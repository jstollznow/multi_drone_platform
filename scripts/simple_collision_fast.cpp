//
// Created by jacob on 19/5/20.
//

#include "user_api.h"

int main() {
    mdp::initialise(10, "simple_collision");

    auto drones = mdp::get_all_rigidbodies();
    if (drones.size() > 1) {
        mdp::cmd_takeoff(drones[0]);
        mdp::cmd_takeoff(drones[1]);

        mdp::sleep_until_idle(drones[0]);
        mdp::sleep_until_idle(drones[1]);
        mdp::position_msg posMsg;
        posMsg.keepHeight = true;
        posMsg.relative = false;
//        for high speed
        posMsg.duration = 2.0;
//        for low speed
//        posMsg.duration = 5.0;
        posMsg.position = {1.0, -1.0, 0.0};
        mdp::sleep_until_idle(drones[0]);
        mdp::set_drone_position(drones[0], posMsg);

        posMsg.position = {1.0, 1.0, 0.0};
        mdp::sleep_until_idle(drones[1]);
        mdp::set_drone_position(drones[1], posMsg);

        mdp::sleep_until_idle(drones[0]);
        posMsg.position = {1.0, 1.0, 0.0};
        mdp::set_drone_position(drones[0], posMsg);

        mdp::sleep_until_idle(drones[1]);
        posMsg.position = {1.0, -1.0, 0.0};
        mdp::set_drone_position(drones[1], posMsg);

        mdp::sleep_until_idle(drones[0]);
        mdp::sleep_until_idle(drones[1]);

        mdp::cmd_land(drones[0]);
        mdp::cmd_land(drones[1]);

        mdp::sleep_until_idle(drones[0]);
        mdp::sleep_until_idle(drones[1]);
    }

    mdp::terminate();

    return 0;
}