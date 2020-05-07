//
// Created by jacob on 7/5/20.
//

#include "user_api.h"
#include "ros/ros.h"


int main() {
    mdp::initialise(10, "pos");
    auto drones = mdp::get_all_rigidbodies();

    if (drones.size() > 0) {
        mdp::cmd_takeoff(drones[0], 0.5);

        mdp::sleep_until_idle(drones[0]);

        mdp::position_msg myMsg;
        myMsg.duration = 10.0;
        myMsg.keepHeight = true;
        myMsg.relative = true;
        myMsg.position = {4.5, 4.5, 0};
        mdp::set_drone_position(drones[0], myMsg);

        mdp::sleep_until_idle(drones[0]);
        myMsg.duration = 15.0;
        myMsg.position = {-4.5, -4.5, 4.5};
        mdp::set_drone_position(drones[0], myMsg);

        mdp::sleep_until_idle(drones[0]);

        mdp::go_to_home(drones[0], 10.0);
    }

    mdp::terminate();
}