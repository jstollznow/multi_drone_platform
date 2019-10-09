#pragma once

#include <map>
#include "../objects/rigidBody.h"

#include "../wrappers/cflie.cpp"
#include "../wrappers/tello.cpp"
#include "../wrappers/vflie.cpp"

std::map<const char*, unsigned int> drone_type_map = {
    {"cflie", 0},
    {"tello", 1},
    {"vflie", 2}
};

namespace mdp_wrappers{
rigidBody* createNewRigidbody(std::string pTag)
{
    std::string DroneType = pTag.substr(0, pTag.find_last_of('_'));
    switch(drone_type_map[DroneType.c_str()]) {
        case 0: {return (rigidBody*)(new cflie(pTag));}
        case 1: {return (rigidBody*)(new tello(pTag));}
        case 2: {return (rigidBody*)(new vflie(pTag));}
        default: return nullptr;
    }
}
}
