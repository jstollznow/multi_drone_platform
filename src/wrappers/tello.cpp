#include "../objects/rigidBody.h"

class tello : private rigidBody
{
public:
    tello(std::string tag):rigidBody(tag) {}
    ~tello() {}

protected:
    virtual void wrapperControlLoop() override {};
    virtual void velocity(geometry_msgs::Vector3 vel, float duration) override {}
    virtual void position(geometry_msgs::Point pos, float duration) override {}
    void land() override
    {

    }
    void emergency() override
    {

    }
};