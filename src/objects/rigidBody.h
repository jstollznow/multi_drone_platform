#pragma once
#include <string>
#include <vector>
#include "ros/ros.h"
#include "nodeData.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "multi_drone_platform/inputData.h"
#include "multi_drone_platform/inputAPI.h"


struct returnPos{
    geometry_msgs::Vector3 position;
    float yaw;
    float duration;
};
struct returnVel{
    geometry_msgs::Vector3 velocity;
    float yawRate;
    float duration;
};
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
        float commandDuration;

        // Velocity handles
        // Twist:
        // linear- m/s
        // angular euler angles (rad/s)
        //      x: roll rate (cw about north axis)
        //      y: pitch rate (cw about east axis)
        //      z: yaw rate (cw about down axis)
        // typically twist.angular are about fixed axes
        // gimbal lock at 90 degrees pitch
        geometry_msgs::Twist desVel;
        geometry_msgs::Twist currVel;

        // Position handles
        geometry_msgs::Pose desPos;
        geometry_msgs::Pose currPos;
        
        geometry_msgs::Vector3 homePos;
    
        node_data moCapNode;

        void initialise();
        void calcVel();
        float getYaw(geometry_msgs::Pose& pos);
        geometry_msgs::Vector3 vec3PosConvert(geometry_msgs::Pose& pos);
    protected:
        virtual void wrapperControlLoop() = 0;
        virtual void velocity(multi_drone_platform::inputData vel, float duration) = 0;
        virtual void position(multi_drone_platform::inputData pos, float duration) = 0;
    public: 

        
    
        rigidBody(std::string tag, bool controllable = false);
        ~rigidBody();
        
        bool getControllable();

        returnPos getCurrPos();
        returnVel getCurrVel();

        returnPos getDesPos();
        void rigidBody::setDesPos(geometry_msgs::Vector3 pos, float yaw, float duration);

        returnVel getDesVel();
        void setDesVel(geometry_msgs::Vector3 vel, float yawRate, float duration);

        geometry_msgs::Vector3 getHomePos();
        void setHomePos(geometry_msgs::Vector3 pos);

        void addMotionCapture(const geometry_msgs::PoseStamped& msg);
        geometry_msgs::PoseStamped getMotionCapture();

        // control loop, whatever else needs to be done each time
        // safeguarding
        void update(std::vector<rigidBody*>& rigidBodies);


        
};

// ouir control loop, please run
// class cflie : public rigidBody
// {

//     virtual void wrapperControlLoop() override
//     {
//         getCurrPos();getDesPos()
//     }
// };