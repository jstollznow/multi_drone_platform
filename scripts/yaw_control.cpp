//
// Created by jacob on 9/7/20.
//

#include "user_api.h"
#include "ros/ros.h"

int main() {
    mdp::initialise(10, "vel");
    auto drones = mdp::get_all_rigidbodies();
    if (drones.size() > 0) {
        auto drone = drones[0];
        mdp::cmd_takeoff(drone, 0.5);
        mdp::sleep_until_idle(drone);

        mdp::position_msg initialPos;
        initialPos.relative = true;
        initialPos.keepHeight = true;
        initialPos.yaw = 450;
        initialPos.position = {0.0, 0.0, 0.0};
        initialPos.duration = 3.0;
        mdp::set_drone_position(drone, initialPos);
        mdp::sleep_until_idle(drone);

        initialPos.relative = true;
        initialPos.keepHeight = true;
        initialPos.yaw = 135;
        initialPos.position = {-1.0, 0.0, 0.0};
        initialPos.duration = 3.0;
        mdp::set_drone_position(drone, initialPos);
        mdp::sleep_until_idle(drone);


        mdp::velocity_msg myMsg;
        myMsg.duration = 3.0;
        myMsg.keepHeight = true;
        myMsg.relative = false;
        myMsg.yawRate = -70;
        myMsg.velocity = {0.1, -0.1, 0};
        mdp::set_drone_velocity(drone, myMsg);
        mdp::sleep_until_idle(drone);

        mdp::velocity_msg revMsg;
        revMsg.duration = 2.0;
        revMsg.keepHeight = true;
        revMsg.relative = false;
        revMsg.yawRate = 25;
        revMsg.velocity = {0.5, 0, 0};
        mdp::set_drone_velocity(drone, revMsg);
        mdp::sleep_until_idle(drone);

        mdp::go_to_home(drone, 2.0);
        mdp::sleep_until_idle(drone);

    }
    mdp::terminate();

    return 0;

}
