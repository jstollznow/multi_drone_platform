#include "../../include/user_api.h"
#include <stdio.h>
#include <iostream>



int main(int argc, char** argv)
{
    mdp_api::initialise(100);

    int a;
    scanf("Enter a number to continue: %d", &a);

    auto drones = mdp_api::get_all_rigidbodies();
    printf("drones:\n", drones.size());
    for (mdp_api::id& id : drones) {
        printf("\t%d\t:\t%s\n", id.numeric_id, id.name.c_str());
    }

    if (drones.size() > 0){

        printf("Taking off\n");
        mdp_api::cmd_takeoff(drones[0]);

        scanf("%d", &a);
        while(a!=9)
        {
            scanf("%d", &a);
        }

        printf("Next Waypoint\n");
    
        // mdp_api::cmd_land(drones[0]);
        // mdp_api::set_drone_position(drones[0], myData.x + 0.5, myData.y - 0.5, myData.z - 0.7, 5.0f);
        //mdp_api::set_drone_position(drones[0], 0.0f, 0.0f, 1.45f, 5.0f);
        scanf("%d", &a);
        while(a!=0)
        {
            scanf("%d", &a);
        }
        mdp_api::position_data myData = mdp_api::get_body_position(drones[0]);
        //mdp_api::set_drone_position(drones[0], myData.x + 2, myData.y, 1.45f, 5.0f);
        scanf("%d", &a);
        while(a!=9)
        {
            scanf("%d", &a);
        }
        mdp_api::cmd_land(drones[0]);
        while(a!=0)
        {
            scanf("%d", &a);
        }
    }

    mdp_api::terminate();
}