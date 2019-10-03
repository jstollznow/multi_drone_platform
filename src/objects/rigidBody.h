#include <string>
#include <vector>
#include "ros/ros.h"
#include "nodeData.cpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
class rigidBody
{
    private:
        int global_id = 0;
        // platformID
        int platform_id;
        //rigid body tag
        std::string optitrackTag;
        std::vector<geometry_msgs::PoseStamped> motionCapture;
        geometry_msgs::Twist velocity;
        void calcVel();
        float yaw;
    public: 
        node_data myNode;
        rigidBody(std::string tag);
        ~rigidBody();
        geometry_msgs::Twist getVelocity();
        void addMotionCapture(geometry_msgs::PoseStamped msg);
        geometry_msgs::PoseStamped getMotionCapture();

};