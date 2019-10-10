#include "../objects/rigidBody.h"

class vflie : private rigidBody
{
public:
    vflie(std::string tag):rigidBody(tag) 
    {
        ROS_INFO_STREAM("vflie: " << tag.c_str());
        this->currPos.position.x = 1.0f;
        this->currPos.position.y = 2.0f;
        this->currPos.position.z = 3.0f;

        this->currVel.linear.x = 4.0f;
        this->currVel.linear.y = 5.0f;
        this->currVel.linear.z = 6.0f;
    }

    ~vflie() 
    {
        printf("Closing vflie\n");
    }

protected:
    virtual void wrapperControlLoop() override 
    {

    };

    virtual void velocity(geometry_msgs::Vector3 vel, float duration) override 
    {
        ROS_INFO("Setting velocity on vflie");
        this->currVel.linear.x = vel.x;
        this->currVel.linear.y = vel.y;
        this->currVel.linear.z = vel.z;
    }

    virtual void position(geometry_msgs::Point pos, float duration) override 
    {

    }

    void land() override
    {

    }

    void emergency() override
    {

    }

};