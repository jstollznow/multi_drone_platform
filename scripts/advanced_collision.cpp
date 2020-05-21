//
// Created by jacob on 18/5/20.
//
#include <iostream>
#include "user_api.h"
#include "ros/ros.h"

int main() {
    mdp::initialise(10, "pos");

    auto drones = mdp::get_all_rigidbodies();

    for (auto drone : drones) {
        mdp::cmd_takeoff(drone, 1.0, 1.0);

    }

    if (drones.size() > 2) {
        mdp::position_msg posMsg;
        posMsg.keepHeight = true;
        posMsg.relative = false;
        posMsg.duration = 1.5;

        posMsg.position = {1.0, -1.0, 0.0};
        mdp::sleep_until_idle(drones[0]);
        mdp::set_drone_position(drones[0], posMsg);

        posMsg.position = {1.1, 1.0, 0.0};
        mdp::sleep_until_idle(drones[1]);
        mdp::set_drone_position(drones[1], posMsg);

        posMsg.position = {0.9, 1.0, 0.0};
        mdp::sleep_until_idle(drones[2]);
        mdp::set_drone_position(drones[2], posMsg);

        posMsg.position = {1.0, 1.0, 0.0};
        mdp::sleep_until_idle(drones[0]);
        mdp::set_drone_position(drones[0], posMsg);

        posMsg.position = {1.1, -1.0, 0.0};
        mdp::sleep_until_idle(drones[1]);
        mdp::set_drone_position(drones[1], posMsg);

        posMsg.position = {0.9, -1.0, 0.0};
        mdp::sleep_until_idle(drones[2]);
        mdp::set_drone_position(drones[2], posMsg);

        mdp::sleep_until_idle(drones[2]);
        mdp::cmd_land(drones[2],1.0);

        mdp::sleep_until_idle(drones[1]);
        mdp::cmd_land(drones[1],1.0);

        mdp::sleep_until_idle(drones[0]);
        mdp::cmd_land(drones[0],1.0);

        mdp::sleep_until_idle(drones[0]);
        mdp::sleep_until_idle(drones[1]);
        mdp::sleep_until_idle(drones[2]);

        mdp::cmd_takeoff(drones[0], 1);
        mdp::sleep_until_idle(drones[0]);
        mdp::set_drone_position(drones[0], posMsg);
        mdp::sleep_until_idle(drones[0]);
        mdp::go_to_home(drones[0]);
        mdp::sleep_until_idle(drones[0]);

    }

    mdp::terminate();

    return 0;
}

