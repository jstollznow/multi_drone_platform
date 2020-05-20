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
        myMsg.duration = 0.5;
        myMsg.keepHeight = true;
        myMsg.relative = false;
        myMsg.position = {2.5, 2.5, 0};

        for(int i = 0; i < 10; i++) {
            mdp::set_drone_position(drones[0], myMsg);
            if (myMsg.position[0] == 2.5) myMsg.position = {-2.5, -2.5, 0};
            else myMsg.position = {2.5, 2.5, 0};
            mdp::sleep_until_idle(drones[0]);
        }
        mdp::cmd_land(drones[0]);
        mdp::sleep_until_idle(drones[0]);
    }
    mdp::terminate();
}