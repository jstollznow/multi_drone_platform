#pragma once
#include <string>
#include <vector>
#include "ros/ros.h"
#include "nodeData.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "multi_drone_platform/inputData.h"
#include "multi_drone_platform/inputAPI.h"

class rigidBody
{
    private:

        int global_id = 0;
    
        // platformID
        int platform_id;
    
        //rigid body tag
        std::string moCapTag;

        bool controllable;

        std::vector<geometry_msgs::PoseStamped> motionCapture;

        // duration of command
        float commDur;

        // Velocity handles
        multi_drone_platform::inputData desVel;
        multi_drone_platform::inputData currVel;

        // Position handles
        multi_drone_platform::inputData desPos;
        multi_drone_platform::inputData currPos;
        
        geometry_msgs::Vector3 homePos;

        void initialise();
        void calcVel();
        
    public: 

        node_data moCapNode;
        node_data apiNode;
    
        rigidBody(std::string tag);
        ~rigidBody();
        
        multi_drone_platform::inputData getCurrPos();
        multi_drone_platform::inputData getCurrVel();

        multi_drone_platform::inputData getDesPos();
        void setDesPos(multi_drone_platform::inputData pos, float duration);

        multi_drone_platform::inputData getDesVel();
        void setDesVel(multi_drone_platform::inputData vel, float duration);

        geometry_msgs::Vector3 getHomePos();
        void setHomePos(geometry_msgs::Vector3 pos);

        void addMotionCapture(geometry_msgs::PoseStamped msg);
        geometry_msgs::PoseStamped getMotionCapture();

        void APIUpdate(multi_drone_platform::inputAPI msg);
};