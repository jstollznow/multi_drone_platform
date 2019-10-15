#pragma once
#include <string>
#include <vector>
#include "ros/ros.h"
#include "nodeData.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"



// api structures
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

    
    protected:
        int global_id = 0;
    
        // platformID
        int platform_id;
    
        //rigid body tag
        std::string tag;
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
    
        ros::NodeHandle motionHandle;
        ros::Subscriber motionSub;

        ros::NodeHandle droneHandle;
        void initialise();
        bool checkTopicValid (std::string topicName);
        void calcVel();
        float getYaw(geometry_msgs::Pose& pos);
        geometry_msgs::Vector3 vec3PosConvert(geometry_msgs::Pose& pos);
        virtual void wrapperControlLoop() = 0;
        virtual void velocity(geometry_msgs::Vector3 vel, float duration) = 0;
        virtual void position(geometry_msgs::Point pos, float duration) = 0;
        virtual void land() = 0;
        virtual void emergency() = 0;
    public: 

        // static std::map<std::string, int> APIMap = {
        //     {"VELOCITY", 0},    {"POSITION", 1},    {"TAKEOFF", 2},
        //     {"LAND", 3},        {"HOVER", 4},       {"EMERGENCY", 5},
        //     {"SET_HOME", 6},    {"GET_HOME", 7},    {"GOTO_HOME", 8},
        //     {"ORIENTATION", 9}, {"TIME", 10},       {"DRONE_SERVER_FREQ", 11}
        // };
    
        rigidBody(std::string tag, bool controllable = false);

        virtual ~rigidBody();
        
        bool getControllable();
        std::string getName();

        returnPos getCurrPos();
        returnVel getCurrVel();

        returnPos getDesPos();
        void setDesPos(geometry_msgs::Vector3 pos, float yaw, float duration);

        returnVel getDesVel();
        void setDesVel(geometry_msgs::Vector3 vel, float yawRate, float duration);

        geometry_msgs::Vector3 getHomePos();
        void setHomePos(geometry_msgs::Vector3 pos);

        void addMotionCapture(const geometry_msgs::PoseStamped::ConstPtr& msg);
        geometry_msgs::PoseStamped getMotionCapture();

        // control loop, whatever else needs to be done each time
        // safeguarding
        void update(std::vector<rigidBody*>& rigidBodies);

        
        
};