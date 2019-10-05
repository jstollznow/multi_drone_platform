#include <string>
#include <vector>
#include "ros/ros.h"
#include "nodeData.cpp"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "multi_drone_platform/inputData.h"
class rigidBody
{
    private:
        int global_id = 0;
        // platformID
        int platform_id;
        //rigid body tag
        std::string optitrackTag;
        std::vector<geometry_msgs::PoseStamped> motionCapture;

        multi_drone_platform::inputData desVel;
        multi_drone_platform::inputData currVel;
        
        multi_drone_platform::inputData desPos;
        multi_drone_platform::inputData currPos;
        
        geometry_msgs::Vector3 homePos;

        void initialise();
        void calcVel();

    public: 
        node_data myNode;
        rigidBody(std::string tag);
        ~rigidBody();
        
        multi_drone_platform::inputData getCurrPos();
        multi_drone_platform::inputData getCurrVel();

        multi_drone_platform::inputData getDesPos();
        void setDesPos(multi_drone_platform::inputData pos);

        multi_drone_platform::inputData getDesVel();
        void setDesVel(multi_drone_platform::inputData vel);

        multi_drone_platform::inputData getHomePos();
        void setHomePos(multi_drone_platform::inputData pos);

        void addMotionCapture(geometry_msgs::PoseStamped msg);
        geometry_msgs::PoseStamped getMotionCapture();

};