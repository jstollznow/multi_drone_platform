#pragma once
#include <string>
#include <vector>
#include "multi_drone_platform/inputData.h"
class droneWrapper
{
public:
    droneWrapper(){}
    ~droneWrapper(){}
    virtual void velocity(multi_drone_platform::inputData vel, float duration) = 0;
    virtual void position(multi_drone_platform::inputData pos, float duration) = 0;
};
