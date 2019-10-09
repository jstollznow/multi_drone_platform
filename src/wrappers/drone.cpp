#include "../objects/rigidBody.h"


class drone : public rigidBody
{
private:
    int myMember;

public:
    drone(std::string tag):rigidBody(tag){};
    ~drone() {}
    
    void velocity(geometry_msgs::Vector3 vel, float duration) override
    {

    }
    void position(geometry_msgs::Vector3 pos, float duration) override
    {

    }
    void wrapperControlLoop() override
    {

    }
};


