#include "../objects/rigidBody.h"

class testdrone : private rigidBody
{
public:
    testdrone(std::string tag):rigidBody(tag) 
    {
        ROS_INFO_STREAM("test drone: " << tag.c_str());
        this->currPos.position.x = 1.0f;
        this->currPos.position.y = 2.0f;
        this->currPos.position.z = 3.0f;

        this->currVel.linear.x = 4.0f;
        this->currVel.linear.y = 5.0f;
        this->currVel.linear.z = 6.0f;
    }

    ~testdrone() 
    {
        printf("Closing test drone\n");
    }

protected:
    virtual void wrapperControlLoop() override 
    {

    };

    virtual void velocity(geometry_msgs::Vector3 vel, float duration) override 
    {
        ROS_INFO("Setting velocity on test drone");
        this->currVel.linear.x = vel.x;
        this->currVel.linear.y = vel.y;
        this->currVel.linear.z = vel.z;
    }

    virtual void position(geometry_msgs::Point pos, float duration) override 
    {
        ROS_INFO("Setting velocity on test drone");
        this->currPos.position.x = pos.x;
        this->currPos.position.y = pos.y;
        this->currPos.position.z = pos.z;
    }

    void land() override
    {
        ROS_INFO("Called land on test drone");
    }

    void emergency() override
    {
        ROS_INFO("Called emergency on test drone");
    }

};