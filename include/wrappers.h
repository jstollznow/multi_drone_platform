#pragma once

#include <map>
#include <string>
#include "rigidbody.h"

#include "../src/wrappers/object.cpp"
#include "../src/wrappers/cflie.cpp"
#include "../src/wrappers/vflie.cpp"

std::map<std::string, unsigned int> droneTypeMap = {
    {"object", 1},
    {"cflie", 2},
    {"vflie", 3}
};

namespace mdp_wrappers {
bool create_new_rigidbody(std::string pTag, uint32_t id, rigidbody* &pRigidbodyPtr) {
    std::string droneType = pTag.substr(0, pTag.find_first_of('_'));
    switch(droneTypeMap[droneType]) {
        case 1: {pRigidbodyPtr = (rigidbody*)(new object(pTag, id)); return true;}
        case 2: {pRigidbodyPtr = (rigidbody*)(new cflie(pTag, id)); return true;}
        case 3: {pRigidbodyPtr = (rigidbody*)(new vflie(pTag, id)); return true;}
        default: {pRigidbodyPtr = nullptr; return false;}
    }
}
}
