#include "../../include/user_api.h"
#include <stdio.h>






int main(int argc, char** argv)
{
    mdp_api::initialise();

    int a;
    scanf("Enter a number to continue: %d", &a);

    auto drones = mdp_api::get_all_rigidbodies();
    printf("drones:\n", drones.size());
    for (mdp_api::id& id : drones) {
        printf("\t%d\t:\t%s\n", id.numeric_id, id.name.c_str());
    }

    if (drones.size() > 0){
        mdp_api::set_drone_velocity(drones[0], 0.5f, 1.5f, 2.5f, -8.0f);
        mdp_api::set_drone_position(drones[0], 1.2f, 2.2f, 3.2f);

        mdp_api::position_data Res = mdp_api::get_body_position(drones[0]);
        printf("Res DATA: %s %f %f %f %f\n", drones[0].name.c_str(), Res.x, Res.y, Res.z, Res.yaw);
        Res = mdp_api::get_body_velocity(drones[0]);
        printf("Res DATA: %s %f %f %f %f\n", drones[0].name.c_str(), Res.x, Res.y, Res.z, Res.yaw);
        Res = mdp_api::get_home(drones[0]);
        printf("Res DATA: %s %f %f %f %f\n", drones[0].name.c_str(), Res.x, Res.y, Res.z, Res.yaw);
    }

    mdp_api::terminate();
}