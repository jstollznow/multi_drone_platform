#include "droneWrapper.h"


class Crazyflie : public droneWrapper
{
private:
    int myMember;

public:
    Crazyflie();
    ~Crazyflie();
    void velocity(multi_drone_platform::inputData vel, float duration);
    void position(multi_drone_platform::inputData pos, float duration);
};

void Crazyflie::velocity(multi_drone_platform::inputData vel, float duration)
{

}
void Crazyflie::position(multi_drone_platform::inputData pos, float duration)
{

}
