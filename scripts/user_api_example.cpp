#include "user_api.h"
#include <stdio.h>
#include <iostream>



int main(int argc, char** argv)
{
    mdp::initialise(100,  "t2");

    int a;
    scanf("Enter a number to continue: %d", &a);

    auto drones = mdp::get_all_rigidbodies();
    printf("drones: %zu\n", drones.size());
    for (mdp::id& id : drones) {
        printf("\t%d\t:\t%s\n", id.numericID, id.name.c_str());
    }

    if (drones.size() > 0){

        printf("Taking off\n");
        mdp::cmd_takeoff(drones[0]);

        scanf("%d", &a);
        while(a!=9)
        {
            scanf("%d", &a);
        }

        printf("Next Waypoint\n");
    
        // mdp::cmd_land(drones[0]);
        // mdp::set_drone_position(drones[0], myData.x + 0.5, myData.y - 0.5, myData.z - 0.7, 5.0f);
        //mdp::set_drone_position(drones[0], 0.0f, 0.0f, 1.45f, 5.0f);
        scanf("%d", &a);
        while(a!=0)
        {
            scanf("%d", &a);
        }
        mdp::position_data myData = mdp::get_position(drones[0]);
        //mdp::set_drone_position(drones[0], myData.x + 2, myData.y, 1.45f, 5.0f);
        scanf("%d", &a);
        while(a!=9)
        {
            scanf("%d", &a);
        }
        mdp::cmd_land(drones[0]);
        while(a!=0)
        {
            scanf("%d", &a);
        }
    }

    mdp::terminate();
}