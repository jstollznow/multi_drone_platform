#include "../objects/rigidBody.h"


class cflie : public rigidBody
{
private:
    int myMember;

public:
    cflie(std::string tag):rigidBody(tag){};
    ~cflie() {}
    
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


