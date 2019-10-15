#include "../include/user_api.h"
#include <iostream>

int main(int argc, char **argv)
{
    // stage 1
    // mdp_api::initialise();
    // mdp_api::id myDrone = mdp_api::get_all_rigidbodies()[0];
    // mdp_api::position_data myDronePos = mdp_api::get_body_position(myDrone);
    // mdp_api::cmd_takeoff(myDrone);
    // mdp_api::cmd_hover(myDrone);
    std::cout<<"waiting"<<std::endl;
    while (std::cin.get() !='l')
    {
        std::cout<<"waiting"<<std::endl;
    }
    // mdp_api::cmd_land(myDrone);
    

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
    return 0;
}