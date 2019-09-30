#include "../../include/user_api.h"
#include <stdio.h>






int main(int argc, char** argv)
{
    mdp_api::initialise(argc, argv);

    int a;
    scanf("Enter a number to continue: %d", &a);

    mdp_api::set_drone_velocity(0, 0.5f, 1.5f, 2.5f, -8.0f);
    mdp_api::set_drone_position(1, 1.2f, 2.2f, 3.2f);

    mdp_api::position_data Res = mdp_api::get_body_position(0);
    printf("Res DATA: %d %f %f %f %f\n", Res.droneID, Res.x, Res.y, Res.z, Res.yaw);
    Res = mdp_api::get_body_velocity(1);
    printf("Res DATA: %d %f %f %f %f\n", Res.droneID, Res.x, Res.y, Res.z, Res.yaw);

    mdp_api::terminate();
}