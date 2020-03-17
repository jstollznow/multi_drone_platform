#include "user_api.h"
#include "ros/ros.h"


int main() {
    mdp_api::initialise(10);
    auto drones = mdp_api::get_all_rigidbodies();

    if (drones.size() > 0) {
        mdp_api::cmd_takeoff(drones[0], 0.5);

        mdp_api::sleep_until_idle(drones[0]);
        
        mdp_api::velocity_msg myMsg;
        myMsg.duration = 2.0;
        myMsg.keepHeight = true;
        myMsg.relative = true;
        myMsg.velocity = {1.5, 0, 0};
        mdp_api::set_drone_velocity(drones[0], myMsg);

        mdp_api::sleep_until_idle(drones[0]);

        myMsg.velocity = {-1.5, 0, 0};
        mdp_api::set_drone_velocity(drones[0], myMsg);

        mdp_api::sleep_until_idle(drones[0]);

        mdp_api::cmd_land(drones[0]);
    }

    mdp_api::terminate();
}