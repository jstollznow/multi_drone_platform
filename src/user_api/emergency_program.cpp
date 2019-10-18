#include "../../include/user_api.h"



int main(int argc, char** argv)
{
    mdp_api::initialise(100);

    int a;
    scanf("%d", &a);
    while(a!=0)
    {
        scanf("%d", &a);
    }

    auto drones = mdp_api::get_all_rigidbodies();
    for (size_t i = 0; i < drones.size(); i++) {
        mdp_api::cmd_emergency(drones[i]);
    }

    scanf("%d", &a);
    while(a!=9)
    {
        scanf("%d", &a);
    }
    
    mdp_api::terminate();
}