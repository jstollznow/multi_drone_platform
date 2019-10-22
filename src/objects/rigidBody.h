#pragma once
#include <string>
#include <vector>
#include "nodeData.h"
#include "ros/callback_queue.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Imu.h"

#define DEFAULT_QUEUE 10
#define TIMEOUT_GEN 0.1
#define TIMEOUT_HOVER 4


// api structures
struct returnPos{
    ros::Time lastUpdate;
    geometry_msgs::Vector3 position;
    float yaw;
    float duration;
};
struct returnVel{
    ros::Time lastUpdate;
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

        std::string tag;
        bool controllable;

        

        bool timeoutStageOne = true;
        double nextTimeoutGen;

        ros::Time lastUpdate;
        ros::Time lastCommandSet;

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

        ros::AsyncSpinner mySpin;
        ros::CallbackQueue myQueue;

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

        void land(float duration = 5.0);

        void takeoff(float height = 0.25, float duration = 2.0);

        void addToQueue();
};