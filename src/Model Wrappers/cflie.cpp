#include "../objects/rigidBody.h"


class cflie : public rigidBody
{
private:
    int myMember;

public:
    cflie(std::string tag):rigidBody(tag){};
    ~cflie();
    
    void cflie::velocity(multi_drone_platform::inputData vel, float duration) override
    {

    }
    void cflie::position(multi_drone_platform::inputData pos, float duration) override
    {

    }
    void wrapperControlLoop() override
    {

    }
};


