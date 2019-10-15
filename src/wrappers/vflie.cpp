#include "../objects/rigidBody.h"

class vflie : private rigidBody
{
public:
    vflie(std::string tag):rigidBody(tag) {ROS_INFO_STREAM("vflie: " << tag.c_str());}
    ~vflie() {printf("Closing vflie\n");}

protected:
    virtual void wrapperControlLoop() override {};
    virtual void velocity(geometry_msgs::Vector3 vel, float duration) override {}
    virtual void position(geometry_msgs::Point pos, float duration) override {}
    // virtual void acceleration(geometry_msgs::Vector3 acceleration, float duration)
    void land() override
    {

    }
    void emergency() override
    {

    }
};