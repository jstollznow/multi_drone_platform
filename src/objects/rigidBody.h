#pragma once
#include <string>
#include <vector>
#include "nodeData.h"
#include "ros/callback_queue.h"
#include "multi_drone_platform/apiUpdate.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64MultiArray.h"

#define DEFAULT_QUEUE 10
#define TIMEOUT_HOVER 4

// api structures

static std::map<std::string, int> APIMap = {
    {"VELOCITY", 0},    {"POSITION", 1},    {"TAKEOFF", 2},
    {"LAND", 3},        {"HOVER", 4},       {"EMERGENCY", 5},
    {"SET_HOME", 6},    {"GET_HOME", 7},    {"GOTO_HOME", 8},
    {"ORIENTATION", 9}, {"TIME", 10},       {"DRONE_SERVER_FREQ", 11}
};

class rigidBody
{
/* DATA */
    private:
        ros::Subscriber ApiSubscriber;
        ros::Publisher CurrentPosePublisher;
        ros::Publisher DesiredPosePublisher;
        uint32_t NumericID;
        
    protected:
        std::vector<multi_drone_platform::apiUpdate> CommandQueue;
        std::string tag;
        bool controllable;
        bool batteryDead;
        double nextTimeoutGen;
        ros::Time lastUpdate;
        ros::Time lastCommandSet;
        std::vector<geometry_msgs::PoseStamped> motionCapture;

        // velocity handles
        geometry_msgs::Twist DesiredVelocity;
        geometry_msgs::Twist CurrentVelocity;

        // Position handles
        geometry_msgs::Pose DesiredPose;
        geometry_msgs::Pose CurrentPose;
        geometry_msgs::Vector3 HomePosition;
        ros::Subscriber motionSub;
        ros::NodeHandle droneHandle;

    public:
        std::string State = "LANDED";
        ros::AsyncSpinner mySpin;
        ros::CallbackQueue myQueue;
        ros::Publisher ApiPublisher;

/* FUNCTIONS */
    private:
        void calculateVelocity();
        float getYaw(geometry_msgs::Pose& pos);
        void set_state(const std::string& state);
        
    protected:
        void handleCommand();
        void enqueueCommand(multi_drone_platform::apiUpdate command);
        void dequeueCommand();
        void resetTimeout(float timeout = 1.0f);
        // Wrapper Methods
        virtual void onUpdate() = 0;
        virtual void onMotionCapture(const geometry_msgs::PoseStamped::ConstPtr& msg) {};
        virtual void onTakeoff(float height, float duration) = 0;
        virtual void onLand(float duration) = 0;
        virtual void onEmergency() = 0;
        virtual void onSetPosition(geometry_msgs::Vector3 pos, float yaw, float duration, bool isRelative) = 0;
        virtual void onSetVelocity(geometry_msgs::Vector3 vel, float yawrate, float duration, bool isRelative) = 0;

    public:
        rigidBody(std::string tag, uint32_t id);
        virtual ~rigidBody();

        void setID(uint32_t id);

        bool getControllable();
        std::string getName();

        geometry_msgs::Vector3 predictCurrentPosition();
        double predictCurrentYaw();

        void setDesPos(geometry_msgs::Vector3 pos, float yaw, float duration, bool relativeXY, bool relativeZ);
        void setDesVel(geometry_msgs::Vector3 vel, float yawRate, float duration, bool relativeXY, bool relativeZ);

        geometry_msgs::Vector3 getHomePos();
        void setHomePos(geometry_msgs::Vector3 pos, bool relative);

        void addMotionCapture(const geometry_msgs::PoseStamped::ConstPtr& msg);
        geometry_msgs::PoseStamped getMotionCapture();

        void update(std::vector<rigidBody*>& rigidBodies);
        void apiCallback(const multi_drone_platform::apiUpdate& msg);
        void emergency();
        void land(float duration = 5.0f);
        void takeoff(float height = 0.25f, float duration = 2.0f);
        void hover(float duration);
        void addToQueue();

};