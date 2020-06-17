//
// Created by jacob on 4/6/20.
//
#include "user_api.h"

int main() {
    mdp::initialise(10, "complex_collision");
    auto drones = mdp::get_all_rigidbodies();
    if (drones.size() >= 6) {
        mdp::cmd_takeoff(drones[0], 1);
        mdp::cmd_takeoff(drones[1], 1);
        mdp::cmd_takeoff(drones[2], 1);
        mdp::cmd_takeoff(drones[3], 1);
        mdp::cmd_takeoff(drones[4], 1);
        mdp::cmd_takeoff(drones[5], 1);

        mdp::sleep_until_idle(drones[0]);
        mdp::cmd_hover(drones[0], 30);

        mdp::sleep_until_idle(drones[1]);
        mdp::cmd_hover(drones[1], 30);

        mdp::sleep_until_idle(drones[3]);
        mdp::cmd_hover(drones[3], 30);

        mdp::sleep_until_idle(drones[4]);
        mdp::cmd_hover(drones[4], 30);

        mdp::sleep_until_idle(drones[5]);
        mdp::cmd_hover(drones[5], 30);

        mdp::position_msg endMsg;
        endMsg.position = {-1.5,-1.5,1};
        endMsg.duration = 5.0;
        endMsg.relative = false;
        endMsg.keepHeight = false;

        mdp::sleep_until_idle(drones[2]);
        mdp::set_drone_position(drones[2], endMsg);

        mdp::sleep_until_idle(drones[2]);
        mdp::cmd_land(drones[2]);
        mdp::cmd_land(drones[3]);
        mdp::cmd_land(drones[4]);
        mdp::cmd_land(drones[5]);
        mdp::cmd_land(drones[1]);
        mdp::cmd_land(drones[0]);

        mdp::sleep_until_idle(drones[0]);
        mdp::sleep_until_idle(drones[1]);
        mdp::sleep_until_idle(drones[2]);
        mdp::sleep_until_idle(drones[3]);
        mdp::sleep_until_idle(drones[4]);
        mdp::sleep_until_idle(drones[5]);
    }
    mdp::terminate();

    return 0;
}


