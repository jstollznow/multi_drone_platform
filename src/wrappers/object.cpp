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

    void onSetPosition(geometry_msgs::Vector3 pos, float yaw, float duration) override
    {

    }

    void onMotionCapture(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {

    }
    
    void onUpdate() override
    {

    }

    void onTakeoff(float height) override
    {

    }

    void onLand() override
    {

    }

    void onEmergency() override
    {

    }
};


