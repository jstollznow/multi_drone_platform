#pragma once
#include <string>
#include <vector>
#include "ros/ros.h"
#include "nodeData.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Imu.h"

#define DEFAULT_QUEUE 10
#define TIMEOUT_GEN 0.1
#define TIMEOUT_HOVER 4


// api structures
struct returnPos{
    // std_msgs::Header stamp;
    geometry_msgs::Vector3 position;
    float yaw;
    float duration;
};
struct returnVel{
    // std_msgs::Header stamp;
    geometry_msgs::Vector3 velocity;
    float yawRate;
    float duration;
};

class rigidBody
{
    private:
        void calcVel();
        float getYaw(geometry_msgs::Pose& pos);
        geometry_msgs::Vector3 vec3PosConvert(geometry_msgs::Pose& pos);
        void set_state(const std::string& state);
        
    protected:
        //rigid body tag
        std::string tag;
        bool controllable;

        bool timeoutStageOne = true;
        double nextTimeoutGen;

        std::vector<geometry_msgs::PoseStamped> motionCapture;

        // duration of command
        float commandDuration;

        geometry_msgs::Twist desVel;
        geometry_msgs::Twist currVel;

        // Position handles
        geometry_msgs::Pose desPos;
        geometry_msgs::Pose currPos;
        
        geometry_msgs::Vector3 homePos;
    
        ros::Subscriber motionSub;

        ros::NodeHandle droneHandle;

        void resetTimeout(float timeout = TIMEOUT_GEN);

        // Wrapper Methods

        virtual void onUpdate() = 0;
        virtual void onMotionCapture(const geometry_msgs::PoseStamped::ConstPtr& msg) {};
        virtual void onTakeoff(float height, float duration) = 0;
        virtual void onLand(float duration) = 0;
        virtual void onEmergency() = 0;

        virtual void onSetPosition(geometry_msgs::Vector3 pos, float yaw, float duration) = 0;

    public:
        std::string State = "LANDED";
        bool StateIsDirty = true;

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

        void update(std::vector<rigidBody*>& rigidBodies);

        void emergency();

        void land();

        void takeoff(float height = 0.25, float duration = 2.0);
};