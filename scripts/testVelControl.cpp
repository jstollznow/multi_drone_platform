#include "user_api.h"
#include "ros/ros.h"


int main() {
    mdp::initialise(10, "vel");
    auto drones = mdp::get_all_rigidbodies();

    if (drones.size() > 0) {
        mdp::cmd_takeoff(drones[0], 0.5);
        mdp::sleep_until_idle(drones[0]);
        mdp::position_msg initialPos;
        initialPos.relative = false;
        initialPos.keepHeight = true;
        initialPos.position = {-2.0, -2.0, 0.0};
        initialPos.duration = 3.0;
        mdp::set_drone_position(drones[0], initialPos);
        mdp::sleep_until_idle(drones[0]);

        mdp::velocity_msg myMsg;
        myMsg.duration = 2.0;
        myMsg.velocity = {2.0, 2.0, 0};
        for(int i = 0; i < 10; i++) {
            mdp::set_drone_velocity(drones[0], myMsg);
            if (myMsg.velocity[0] == 2.0) myMsg.velocity = {-2.0, -2.0, 0};
            else myMsg.velocity = {2.0, 2.0, 0};
            mdp::sleep_until_idle(drones[0]);
        }
        mdp::cmd_land(drones[0]);
        mdp::sleep_until_idle(drones[0]);
        auto pos = mdp::get_position(drones[0]);
        std::cout << "Difference between start and end: ["
                  << std::abs(2.0 + pos.x) << ", " <<
                  std::abs(2.0 + pos.y) << "]" << std::endl;
    }
    mdp::terminate();
}