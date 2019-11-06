#include "../objects/rigidBody.h"

class object : public rigidBody
{
private:

public:
    object(std::string tag) : rigidBody(tag)
    {

    };

    ~object() 
    {

    }

    void onSetPosition(geometry_msgs::Vector3 pos, float yaw, float duration, bool isRelative) override
    {

    }

    void onSetVelocity(geometry_msgs::Vector3 vel, float yawrate, float duration, bool isRelative) override
    {

    }

    void onMotionCapture(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {

    }
    
    void onUpdate() override
    {

    }

    void onTakeoff(float height, float duration) override
    {

    }

    void onLand(float duration) override
    {

    }

    void onEmergency() override
    {

    }
};


