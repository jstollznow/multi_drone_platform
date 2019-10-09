#include "../objects/rigidBody.h"


class cflie : public rigidBody
{
private:
    int myMember;

public:
    cflie(std::string tag):rigidBody(tag){};
    ~cflie();
    
    void cflie::velocity(geometry_msgs::Vector3 vel, float duration) override
    {

    }
    void cflie::position(geometry_msgs::Vector3 pos, float duration) override
    {

    }
    void wrapperControlLoop() override
    {

    }
    sensor_msgs::Imu getIMU() override
    {
        
    }
};


