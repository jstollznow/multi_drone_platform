#pragma once
#include <string>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64MultiArray.h"
#include "../src/debug/logger/logger.h"
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

class mdp_timer {
private:
    bool isStage1Timeout = false;
    bool timerIsActive = false;
    double timeoutTime{};

public:
    bool is_active() const;
    void close_timer();
    void reset_timer(double duration, bool Stage1Timeout = false);
    bool has_timed_out();
    bool is_stage_timeout() const;

};

/**
 * @brief The base class for all drone wrappers.
 * the rigidbody class should be inherited
 * from when creating drone wrappers, these wrappers are then placed inside the /wrappers/ folder. All files within
 * the /wrappers/ folder will be automatically included in future compiles of the multi-drone platform drone_server.
 * When creating a new drone wrapper, ensure the class name is the same as the file name, this name will be the recognisable tag for
 * the new drone. i.r. for crazyflies, the wrapper file is cflie.cpp and the class definition is class cflie : public rigidbody {...}.
 *
 * object.cpp is a template file for drone wrappers and exists within the /wrappers/ folder
 * @ingroup public_api
 */
class rigidbody {
    enum flight_state {
        UNKNOWN,
        LANDED,
        HOVERING,
        MOVING,
        SHUTTING_DOWN,
        DELETED
    };

    friend class drone_server;
/* DATA */
    private:
        ros::Subscriber apiSubscriber;
        ros::Publisher logPublisher;
        ros::Publisher batteryPublisher;
        ros::Publisher currentPosePublisher;
        ros::Publisher currentTwistPublisher;
        ros::Publisher desiredPosePublisher;
        ros::Publisher desiredTwistPublisher;
        uint32_t numericID;
        bool shutdownHasBeenCalled = false;
        ros::AsyncSpinner mySpin;
        ros::CallbackQueue myQueue;
        ros::Publisher apiPublisher;
        flight_state state = flight_state::UNKNOWN; /** The current state of the rigidbody */
        mdp_timer hoverTimer;
        double declaredStateEndTime = 0.0;
        
    protected:
        std::vector<multi_drone_platform::api_update> commandQueue;
        std::string tag;

        bool batteryDying = false;

        multi_drone_platform::api_update lastRecievedApiUpdate;
        ros::Time timeOfLastApiUpdate;
        ros::Time lastUpdate;
        std::queue<geometry_msgs::PoseStamped> motionCapture;

        // velocity handles
        geometry_msgs::Twist desiredVelocity;
        geometry_msgs::Twist currentVelocity;

        // Position handles
        geometry_msgs::Pose desiredPose;
        geometry_msgs::Pose currentPose;
        geometry_msgs::Vector3 homePosition;
        ros::Subscriber motionSubscriber;
        ros::NodeHandle droneHandle;


    /* FUNCTIONS */
    private:
        void calculate_velocity();
        static double vec3_distance(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b);

        /* this function declares that we are expecting the drone to enter this state very soon. This expected overrides the physical state for the next 100ms or so.
         * This is so that state immediately changes when a call to set position for instance is made (and so that wait_till_idle on the user api is not skipped over) */
        void declare_expected_state(flight_state inputState);
        void set_state(const flight_state& state);
        const flight_state& get_state() const;
        static std::string get_flight_state_string(flight_state input);
        void update_current_flight_state();

        void publish_physical_state() const;

        void handle_command();
        void enqueue_command(multi_drone_platform::api_update command);
        void dequeue_command();
        void do_stage_1_timeout();
        bool is_msg_different(multi_drone_platform::api_update msg);

        void set_desired_position(geometry_msgs::Vector3 pos, float yaw, float duration, bool relativeXY, bool relativeZ);
        void set_desired_velocity(geometry_msgs::Vector3 vel, float yawRate, float duration, bool relativeXY, bool relativeHeight);

        void add_motion_capture(const geometry_msgs::PoseStamped::ConstPtr& msg);
        geometry_msgs::PoseStamped get_motion_capture();

        void update(std::vector<rigidbody*>& rigidBodies);
        void api_callback(const multi_drone_platform::api_update& msg);

        void emergency();
        void land(float duration = 5.0f);
        void takeoff(float height = 0.25f, float duration = 3.0f);
        void hover(float duration);
        void go_home(float yaw = 0.0f, float duration = 4.0f, float height = 0.0f);
        void shutdown();

        geometry_msgs::Vector3 get_home_coordinates();
        void set_home_coordiates(geometry_msgs::Vector3 pos, bool relative);

    protected:
        /**
         * The log function is called to output any text for debugging or user feedback purposes depending
         * the logger level. It posts to both the relevant ROS output stream (INFO, WARN, ERROR) and also to
         * a log topic for use in the debugging windows.
         * @param msgType msgType contains the level of the log. This can be one of four levels including
         * INFO, DEBUG, WARN, ERROR.
         * @param message message contains the text message
         */
        void log(logger::log_type msgType, std::string message);
        
        // Wrapper Methods
        /**
         * The on_update function is called at the update rate defined in the drone-server, by default this is 100Hz
         * but is modifiable by a user application
         */
        virtual void on_update() = 0;

        /**
         * on_motion_capture is called whenever the drone receives a new motion capture frame.
         * @param msg the motion capture frame presented as a ros PoseStamped. header contains time stamp information, pose
         * contains position data (cartesian from origin) and orientation data (as quaternion).
         */
        virtual void on_motion_capture(const geometry_msgs::PoseStamped::ConstPtr& msg) {};

        /**
         * on_takeoff is called whenever a takeoff command is to be sent to the drone.
         * @param height height contains the desired height after takeoff (meters)
         * @param duration contains how long the takeoff sequence should take (seconds)
         */
        virtual void on_takeoff(float height, float duration) = 0;

        /**
         * on_takeoff is called whenever a land command is to be sent to the drone. The landing zone is directly below the
         * drones current position, retrievable through a call to predict_current_position().
         * @param duration contains how long the land sequence should take (seconds)
         * @see predict_current_position
         */
        virtual void on_land(float duration) = 0;

        /**
         * on_emergency is called whenever an emergency call is sent to the drone. At this point the drone should stop
         * all operation in hopes of preventing damage. It is not expected that the drone continues flight after this call.
         */
        virtual void on_emergency() = 0;

        /**
         * on_set_position is called whenever the drone should move to a new location. this corresponds to a call to
         * set_drone_position in user_api.h.
         * @param pos the desired position for the drone to move to (modified by value of isRelative).
         * @param yaw the desired yaw of the drone upon reaching the desired position (degrees).
         * @param duration the time it should take for the drone to reach this position (seconds).
         * @param isRelative If true, pos refers to a position relative to the drones current position. If false, pos
         * refers to an absolute position.
         * @see set_drone_position
         */
        virtual void on_set_position(geometry_msgs::Vector3 pos, float yaw, float duration, bool isRelative) = 0;

        /**
         * on_set_velocity is called whenever the drone should move with a desired velocity. This corresponds to a call to
         * set_drone_velocity in user_api.h
         * @param vel the desired velocity for the drone (xyz meters per second)
         * @param yawrate the desired yawrate for the drone (degrees per second)
         * @param duration TODO: what is duration in this case?
         * @param relativeHeight TODO: how does the relative height thing even work, its a velocity not an abs height in meters
         */
        virtual void on_set_velocity(geometry_msgs::Vector3 vel, float yawrate, float duration, bool relativeHeight) = 0;

    public:
        /**
         * The base constructor for a rigidbody
         * @param tag the string name of the drone (same as class name)
         * @param id the numeric id of the drone
         */
        rigidbody(std::string tag, uint32_t id);

        /**
         * The base destructor for the rigidbody. Note: ROS is active when this is called
         */
        virtual ~rigidbody();

        /**
         * returns the name of the rigidbody
         * @return string name
         */
        std::string get_name();

        /**
         * predicts the current position of the rigidbody based upon its last known location and its velocity
         * @return a geometry_msgs::Vector3 depicting the rigidbody's predicted location
         */
        geometry_msgs::Vector3 predict_current_position();

        /**
         * perdicts the current yaw of the rigidbody based upon its last known yaw and yawrate
         * @return the predicted yaw
         */
        double predict_current_yaw();

};