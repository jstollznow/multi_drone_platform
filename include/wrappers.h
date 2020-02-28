#pragma once

#include <map>
#include <string>
#include "rigidBody.h"

#include "../src/wrappers/object.cpp"
#include "../src/wrappers/cflie.cpp"
#include "../src/wrappers/vflie.cpp"

std::map<std::string, unsigned int> drone_type_map = {
    {"object", 1},
    {"cflie", 2},
    {"vflie", 3}
};

namespace mdp_wrappers{
bool createNewRigidbody(std::string pTag, uint32_t id, rigidBody* &pRigidbodyPtr)
{
    std::string DroneType = pTag.substr(0, pTag.find_first_of('_'));
    switch(drone_type_map[DroneType]) {
        case 1: {pRigidbodyPtr = (rigidBody*)(new object(pTag, id)); return true;}
        case 2: {pRigidbodyPtr = (rigidBody*)(new cflie(pTag, id)); return true;}
        case 3: {pRigidbodyPtr = (rigidBody*)(new vflie(pTag, id)); return true;}
        default: {pRigidbodyPtr = nullptr; return false;}
    }
}
}
