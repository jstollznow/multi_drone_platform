#pragma once
#include <string>
#include <vector>
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64MultiArray.h"
#include "logger.h"
#include "multi_drone_platform/api_update.h"

#define DEFAULT_QUEUE 10
#define TIMEOUT_HOVER 20

// api structures

static std::map<std::string, int> apiMap = {
    {"VELOCITY", 0},    {"POSITION", 1},    {"TAKEOFF", 2},
    {"LAND", 3},        {"HOVER", 4},       {"EMERGENCY", 5},
    {"SET_HOME", 6},    {"GET_HOME", 7},    {"GOTO_HOME", 8},
    {"ORIENTATION", 9}, {"TIME", 10},       {"DRONE_SERVER_FREQ", 11}
};

class rigidbody {
/* DATA */
    private:
    ros::Subscriber apiSubscriber;
    ros::Publisher logPublisher;
    ros::Publisher currentPosePublisher;
    ros::Publisher desiredPosePublisher;
    uint32_t numericID;
        
    protected:
    std::vector<multi_drone_platform::api_update> commandQueue;
    std::string tag;
    bool controllable;

    bool batteryDying;

    double nextTimeoutGen;
    multi_drone_platform::api_update lastRecievedApiUpdate;
    ros::Time timeOfLastApiUpdate;
    ros::Time lastUpdate;
    ros::Time lastCommandSet;
    std::vector<geometry_msgs::PoseStamped> motionCapture;

    // velocity handles
    geometry_msgs::Twist desiredVelocity;
    geometry_msgs::Twist currentVelocity;

    // Position handles
    geometry_msgs::Pose desiredPose;
    geometry_msgs::Pose currentPose;
    geometry_msgs::Vector3 homePosition;
    ros::Subscriber motionSubscriber;
    ros::NodeHandle droneHandle;

    public:
    std::string state = "LANDED";
    ros::AsyncSpinner mySpin;
    ros::CallbackQueue myQueue;
    ros::Publisher apiPublisher;

/* FUNCTIONS */
    private:
    
    void calculate_velocity();
    double vec3_distance(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b);
    float get_yaw(geometry_msgs::Pose& pos);
    void set_state(const std::string& state);
    void log(logger::log_type msgType, std::string message);
        
    protected:
    void handle_command();
    void enqueue_command(multi_drone_platform::api_update command);
    void dequeue_command();
    void reset_timeout(float timeout = 1.0f);
    bool is_msg_different(multi_drone_platform::api_update msg);

    // Wrapper Methods
    virtual void on_update() = 0;
    virtual void on_motion_capture(const geometry_msgs::PoseStamped::ConstPtr& msg) {};
    virtual void on_takeoff(float height, float duration) = 0;
    virtual void on_land(float duration) = 0;
    virtual void on_emergency() = 0;
    virtual void on_set_position(geometry_msgs::Vector3 pos, float yaw, float duration, bool isRelative) = 0;
    virtual void on_set_velocity(geometry_msgs::Vector3 vel, float yawrate, float duration, bool isRelative) = 0;

    public:
    rigidbody(std::string tag, uint32_t id);
    virtual ~rigidbody();

    bool get_controllable();
    std::string get_name();

    geometry_msgs::Vector3 predict_current_position();
    double predict_current_yaw();

    void set_desired_position(geometry_msgs::Vector3 pos, float yaw, float duration, bool relativeXY, bool relativeZ);
    void set_desired_velocity(geometry_msgs::Vector3 vel, float yawRate, float duration, bool relativeXY, bool relativeZ);

    geometry_msgs::Vector3 get_home_coordinates();
    void set_home_coordiates(geometry_msgs::Vector3 pos, bool relative);

    void add_motion_capture(const geometry_msgs::PoseStamped::ConstPtr& msg);
    geometry_msgs::PoseStamped get_motion_capture();

    void update(std::vector<rigidbody*>& rigidBodies);
    void api_callback(const multi_drone_platform::api_update& msg);

    void emergency();
    void land(float duration = 5.0f);
    void takeoff(float height = 0.25f, float duration = 2.0f);
    void hover(float duration);

};