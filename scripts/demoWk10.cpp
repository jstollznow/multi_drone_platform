#include "user_api.h"
#include <iostream>

int main(int argc, char **argv)
{
    // stage 1
    mdp::initialise(100, "d10");


    auto myDrones = mdp::get_all_rigidbodies();
    // mdp::position_data myDronePos = mdp::get_body_position(myDrone);
    // mdp::cmd_takeoff(myDrone);
    std::cout << "Drone is " << myDrones[0].name.c_str() <<std::endl;
    mdp::cmd_takeoff(myDrones[0]);
    while (std::cin.get() !='\n');
    mdp::cmd_land(myDrones[0]);
    

    // // stage 2
    // mdp::initialise();
    // mdp::id droneA = mdp::get_all_rigidbodies()[0];
    // mdp::cmd_takeoff(droneA);
    // mdp::cmd_hover(droneA);

    // mdp::id droneB = mdp::get_all_rigidbodies()[1];
    // mdp::cmd_takeoff(droneB);
    // mdp::cmd_hover(droneB);
    // while (std::cin.get() !='/n');
    // mdp::cmd_land(droneA);
    // mdp::cmd_land(droneB);


    // // stage 3
    // mdp::initialise();
    // mdp::id myDrone = mdp::get_all_rigidbodies()[0];
    // mdp::cmd_takeoff(myDrone);
    // mdp::set_drone_position(myDrone,0,0,1,3);
    // while (std::cin.get() !='/n');
    // mdp::cmd_land(myDrone);
    mdp::terminate();
    return 0;
}