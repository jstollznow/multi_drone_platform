#include "user_api.h"
#include <iostream>

int main(int argc, char **argv)
{
    // stage 1
    mdp::initialise(100);


    auto myDrones = mdp::get_all_rigidbodies();
    // mdp_api::position_data myDronePos = mdp_api::get_body_position(myDrone);
    // mdp_api::cmd_takeoff(myDrone);
    std::cout << "Drone is " << myDrones[0].name.c_str() <<std::endl;
    mdp::cmd_takeoff(myDrones[0]);
    while (std::cin.get() !='\n');
    mdp::cmd_land(myDrones[0]);
    

    // // stage 2
    // mdp_api::initialise();
    // mdp_api::id droneA = mdp_api::get_all_rigidbodies()[0];
    // mdp_api::cmd_takeoff(droneA);
    // mdp_api::cmd_hover(droneA);

    // mdp_api::id droneB = mdp_api::get_all_rigidbodies()[1];
    // mdp_api::cmd_takeoff(droneB);
    // mdp_api::cmd_hover(droneB);
    // while (std::cin.get() !='/n');
    // mdp_api::cmd_land(droneA);
    // mdp_api::cmd_land(droneB);


    // // stage 3
    // mdp_api::initialise();
    // mdp_api::id myDrone = mdp_api::get_all_rigidbodies()[0];
    // mdp_api::cmd_takeoff(myDrone);
    // mdp_api::set_drone_position(myDrone,0,0,1,3);
    // while (std::cin.get() !='/n');
    // mdp_api::cmd_land(myDrone);
    mdp::terminate();
    return 0;
}