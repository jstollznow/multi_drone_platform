//
// Created by jacob on 19/5/20.
//

#include <iostream>
#include "user_api.h"
#include "ros/ros.h"

int main() {
    mdp::initialise(10, "pos");

    auto drones = mdp::get_all_rigidbodies();
    if (drones.size() > 1) {
        mdp::cmd_takeoff(drones[0]);
        mdp::cmd_takeoff(drones[1]);

        mdp::sleep_until_idle(drones[0]);
        mdp::sleep_until_idle(drones[1]);
        mdp::position_msg posMsg;
        posMsg.keepHeight = true;
        posMsg.relative = false;
        posMsg.duration = 0.8;

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

