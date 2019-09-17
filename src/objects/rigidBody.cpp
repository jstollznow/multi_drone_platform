#include "rigidBody.h"


rigidBody::rigidBody(std::string tag)
{
    this->platform_id = global_id++;
    this->optitrackTag = tag;
    float init[3] = {0.0, 0.0, 0.0};
    this->velocity = init;
}
rigidBody::~rigidBody()
{
    
}