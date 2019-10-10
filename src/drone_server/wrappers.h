#pragma once

#include <map>
#include <string>
#include "../objects/rigidBody.h"

#include "../wrappers/vflie.cpp"
#include "../wrappers/cflie.cpp"
#include "../wrappers/tello.cpp"
#include "../wrappers/testdrone.cpp"

std::map<std::string, unsigned int> drone_type_map = {
    {"vflie", 1},
    {"cflie", 2},
    {"tello", 3},
    {"testdrone", 4}
};

namespace mdp_wrappers{
bool createNewRigidbody(std::string pTag, rigidBody* &pRigidbodyPtr)
{
    std::string DroneType = pTag.substr(0, pTag.find_last_of('_'));
    switch(drone_type_map[DroneType]) {
        case 1: {pRigidbodyPtr = (rigidBody*)(new vflie(pTag)); return true;}
        case 2: {pRigidbodyPtr = (rigidBody*)(new cflie(pTag)); return true;}
        case 3: {pRigidbodyPtr = (rigidBody*)(new tello(pTag)); return true;}
        case 4: {pRigidbodyPtr = (rigidBody*)(new testdrone(pTag)); return true;}
        default: {pRigidbodyPtr = nullptr; return false;}
    }
}
}
