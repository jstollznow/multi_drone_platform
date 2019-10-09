#pragma once

#include <map>
#include "../objects/rigidBody.h"

#include "../wrappers/drone.cpp"
#include "../wrappers/vflie.cpp"
#include "../wrappers/cflie.cpp"
#include "../wrappers/tello.cpp"

std::map<const char*, unsigned int> drone_type_map = {
    {"drone", 0},
    {"vflie", 1},
    {"cflie", 2},
    {"tello", 3}
};

namespace mdp_wrappers{
rigidBody* createNewRigidbody(std::string pTag)
{
    std::string DroneType = pTag.substr(0, pTag.find_last_of('_'));
    switch(drone_type_map[DroneType.c_str()]) {
        case 0: {return (rigidBody*)(new drone(pTag));}
        case 1: {return (rigidBody*)(new vflie(pTag));}
        case 2: {return (rigidBody*)(new cflie(pTag));}
        case 3: {return (rigidBody*)(new tello(pTag));}
        default: return nullptr;
    }
}
}
