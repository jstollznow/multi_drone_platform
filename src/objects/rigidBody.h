#include <string>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
class rigidBody
{
    private:
        int global_id = 0;
        // platformID
        int platform_id;
        //rigid body tag
        std::string optitrackTag;
        geometry_msgs::Pose lastMotionCapture;
        geometry_msgs::Vector3 velocity;
    public: 
        rigidBody(std::string tag);
        ~rigidBody();
        
};