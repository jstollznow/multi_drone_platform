#pragma once
#include <string>
#include <vector>
#include <queue>
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <std_msgs/Float64.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float64MultiArray.h"
#include "../src/debug/logger/logger.h"
#include "multi_drone_platform/api_update.h"
#include "../src/icp_implementation/icp_object.h"

#define DEFAULT_QUEUE 10
#define TIMEOUT_HOVER 20

/**
 * declares the use of Natnet in motion capture
 */
#define USE_NATNET false

// api structures

/**
 * map representing all drone commands available
 */
static std::map<std::string, int> apiMap = {
    {"VELOCITY", 0},    {"POSITION", 1},    {"TAKEOFF", 2},
    {"LAND", 3},        {"HOVER", 4},       {"EMERGENCY", 5},
    {"SET_HOME", 6},    {"GET_HOME", 7},    {"GOTO_HOME", 8},
    {"ORIENTATION", 9}, {"TIME", 10},       {"DRONE_SERVER_FREQ", 11}
};

/**
 * class used internally to handle timeout timers for each rigidbody
 */
class mdp_timer {
private:
    bool isStage1Timeout = false;
    bool timerIsActive = false;
    double timeoutTime {};

public:
    /**
     * declares the timer as no longer valid
     */
    void close_timer();

    /**
     * restarts the timer with the given duration and an indication of if this is for stage 1 timeout
     * @param duration the duration the timer should run for in seconds
     * @param Stage1Timeout an indication whether this is for stage 1 timeout
     */
    void reset_timer(double duration, bool Stage1Timeout = false);

    /**
     * checks to see if the timer has finished. This will only return true once after timer completion
     * @return if the timer has completed and has not yet been reported
     */
    bool has_timed_out();

    /**
     * checks whether this timer is for stage 2 timout
     * @return if the timer is for stage 2 timeout
     */
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
    /**
     * all available flight states a drone can be in
     */
    enum flight_state {
        UNKNOWN,
        LANDED,
        HOVERING,
        MOVING,
        DELETED
    };

    /**
     * classes who are given direct access to the rigidbody class for a variety of reasons
     */
    friend class drone_server;
    friend class static_physical_management;
    friend class potential_fields;
    friend class icp_impl;

/* DATA */
    private:
        /**
         * ID uniquely identifying the drone on the drone server
         */
        uint32_t numericID;

        /**
         * the drone's tag as given by the user
         */
        std::string tag;

        /**
         * ROS subscriber representing this drone's connection to the user API
         */
        ros::Subscriber apiSubscriber;

        ros::Subscriber motionCaptureSubscriber;

        /**
         * A ROS publisher used by the drone server to contact the above subscriber
         */
        ros::Publisher apiPublisher;

        /**
         * various publishers for logging, pose, velocity, obstacles publishing
         */
        ros::Publisher logPublisher;
        ros::Publisher currentPosePublisher;
        ros::Publisher currentTwistPublisher;
        ros::Publisher desiredPosePublisher;
        ros::Publisher desiredTwistPublisher;
        ros::Publisher obstaclesPublisher;
        ros::Publisher closestObstaclePublisher;

        /**
         * A time point representing the end of the last received command
         */
        ros::Time commandEnd;

        /**
         * boolean which turns true on rigidbody shutdown. When this is set true, the drone will no longer perform commands
         */
        bool shutdownHasBeenCalled = false;

        /**
         * ROS async spinner and callback queue to facilitate asynchronous command execution
         */
        ros::AsyncSpinner mySpin;
        ros::CallbackQueue myQueue;

        /**
         * The current observed flight state of the drone
         */
        flight_state state = flight_state::UNKNOWN;

        /**
         * The drone's timeout timer
         */
        mdp_timer timeoutTimer;

        /**
         * The end point where the declared flight state is no longer applied
         */
        double declaredStateEndTime = 0.0;

        /**
         * A queue holding upcoming API commands to be executed by the rigidbody
         */
        std::vector<multi_drone_platform::api_update> commandQueue;

        /**
         * boolean representing if the rigidbody is a vflie, used in ICP
         */
        bool isVflie = false;

    protected:
        /**
         * boolean representing if the drone is running low on battery charge
         */
        bool batteryDying = false; // @TODO: formalise wrapper drone use of this variable (and cflie)

        /**
         * A full copy of the last received API command, the time of this api update
         */
        multi_drone_platform::api_update lastRecievedApiUpdate;
        ros::Time timeOfLastApiUpdate;

        /**
         * The value and time of the last motion capture frame
         */
        std::queue<geometry_msgs::PoseStamped> motionCapture;
        ros::Time timeOfLastMotionCaptureUpdate;

        /**
         * Velocity handles
         */
        geometry_msgs::Twist desiredVelocity;
        geometry_msgs::Twist currentVelocity;

        /**
         * Position handles
         */
        geometry_msgs::Pose desiredPose;
        geometry_msgs::Pose currentPose;

        /**
         * The current absolute yaw of the rigidbody (not modulated to 360 degrees)
         */
        float absoluteYaw;

        /**
         * The home position of the rigidbody
         */
        geometry_msgs::Vector3 homePosition;

        ros::Publisher batteryPublisher;

        /**
         * The ROS node handle for this drone
         */
        ros::NodeHandle droneHandle;

        /**
         * Velocity limits of this drone. The drone will not exceed these limits in set position or velocity commands.
         */
        struct {
            std::array<double, 2> x;
            std::array<double, 2> y;
            std::array<double, 2> z;
        } velocity_limits;
        double maxVel;

        /**
         * related to traditional APF. in kg
         */
        double mass;

        /**
         * related to velocity AP. in meters
         */
        double restrictedDistance;
        double influenceDistance;

        /**
         * related to safeguarding LiveView feedback. in meters
         */
        double width;
        double height;
        double length;
public:
        icp_object icpObject;

    /* FUNCTIONS */
    private:
        /**
         * calculates and updates the observed velocity of the drone using the last two motion capture frames
         */
        void calculate_velocity();

        /**
         * calculates and updates the absoluteYaw variable on the drone
         */
        void adjust_absolute_yaw();

        /**
         * sets the maxVel variable based upon the given velocity limits
         */
        void set_max_vel();

        /**
         * calculates the distance between the two supplied vector3s
         * @param a the first vector point
         * @param b the second vector point
         * @return the distance between these vector points
         */
        static double vec3_distance(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b);

        /**
         * this function declares that we are expecting the drone to enter this state very soon. This expected overrides the physical state for the next 100ms or so.
         * This is so that state immediately changes when a call to set position for instance is made (and so that wait_till_idle on the user api is not skipped over)
         * @param inputState the flight state to declare
         * @param duration the duration by which to declare this state in seconds
         */
        void declare_expected_state(flight_state inputState, double duration = 0.5);

        /**
         * sets the flight state of the drone
         * @param state the desired flight state
         */
        void set_state(const flight_state& state);

        /**
         * returns the flight state of the drone
         * @return the flight state
         */
        const flight_state& get_state() const;

        /**
         * returns a string representation of the given flight state intended for printing
         * @param input the flight state
         * @return a string representation of the flight state
         */
        static std::string get_flight_state_string(flight_state input);

        /**
         * determines what the current flight state of the drone is based upon it's current velocity. If a state is not
         * being declared then this observed state is set as the drones current flight state.
         */
        void update_current_flight_state();

        /**
         * publishes the pose and velocity of the rigidbody to the ROS topics
         */
        void publish_physical_state() const;

        /**
         * The main function where queued API commands are handled by the drone
         */
        void handle_command();

        void enqueue_command(multi_drone_platform::api_update command);
        void dequeue_command();

        /**
         * performs a stage 1 timeout on the drone
         */
        void do_stage_1_timeout();

        /**
         * determines whether the two supplied api messages are significantly different based on a number of variables
         * @param msg the new message
         * @param last_message the old message
         * @return if they are significantly different
         */
        bool is_msg_different(const multi_drone_platform::api_update& msg, const multi_drone_platform::api_update& last_message) const;

        /**
         * sets the rigidbody's desired position
         * @param pos the absolute position to set to im meters relative to world origin
         * @param yaw the desired yaw in degrees
         * @param duration the duration to reach this position in seconds
         */
        void set_desired_position(geometry_msgs::Vector3 pos, float yaw, float duration);

        /**
         * sets the rigidbody's desired velocity
         * @param vel the velocity in meters per second relative on world coordinates
         * @param yawRate the yawrate in degrees per second
         * @param duration the duration to hold this velocity for in seconds
         */
        void set_desired_velocity(geometry_msgs::Vector3 vel, float yawRate, float duration);

        /**
         * ROS callback to receive motion capture updates
         * @param msg the latest motion capture update
         */
        void add_motion_capture(const geometry_msgs::PoseStamped::ConstPtr& msg);

        /**
         * returns the last motion capture update
         * @return the last motion capture update
         */
        geometry_msgs::PoseStamped get_motion_capture();

        /**
         * main update function called on the rigidbody by the drone server with a list of all active rigidbodies on the platform
         * @param rigidBodies a list of all declared rigidbodies on the platform
         */
        void update(std::vector<rigidbody*>& rigidBodies);

        /**
         * ROS callback to handle api commands
         * @param msg the new api command
         */
        void api_callback(const multi_drone_platform::api_update& msg);

        /**
         * calls emergency on this rigidbody
         */
        void emergency();

        /**
         * calls land on this rigidbody
         * @param duration the duration over which to land in seconds
         */
        void land(float duration);

        /**
         * calls takeoff on this rigidbody
         * @param height the height to go to in meters
         * @param duration the duration it takes to get there in seconds
         */
        void takeoff(float height, float duration);

        /**
         * calls hover on the rigidbody
         * @param duration the duration to hover for in seconds
         */
        void hover(float duration);

        /**
         * calls go to home on the rigidbody
         * @param yaw the desired end yaw of the rigidbody in degrees
         * @param duration the duration to reach home in seconds
         * @param height the desired height upon reaching the home point in meters
         */
        void go_home(float yaw, float duration, float height);

        /**
         * calls shutdown on the rigidbody
         */
        void shutdown();

        /**
         * returns the home position of the rigidbody
         * @return the home position of the rigidbody
         */
        geometry_msgs::Vector3 get_home_coordinates();

        /**
         * sets the home position of the rigidbody
         * @param pos the new home position in meters relative to the world origin
         */
        void set_home_coordiates(geometry_msgs::Vector3 pos);

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

        /**
         * This log_coord function is very similar to log but eases the process of logging a Vector3 object
         * which is used in velocity and position ROS messages
         * @param msgType msgType contains the level of the log. This can be one of four levels including
         * INFO, DEBUG, WARN, ERROR.
         * @param dataLabel dataLabel contains the message to be sent with the set of coordinates
         * @param data data contains the coordinates to be posted
         */
        template <class T>
        void log_coord(logger::log_type msgType, std::string dataLabel, T data);

        /**
         * returns the end yaw when a yawrate of the given duration is conducted on the drone.
         * @param yawrate the input yaw rate to rotate the drone by in degrees
         * @param time_period the time period to apply this yawrate for in seconds
         * @return the end yaw rotation of the drone in degrees
         */
        double get_end_yaw_from_yawrate_and_time_period(double yawrate, double time_period) const;

        /**
         * returns the node handle assosciated to the drone server
         * @return the ros node handle
         */
        ros::NodeHandle get_ros_node_handle() const;

        // Wrapper Methods

        /**
         * This function is called on initialisation of the drone on the platform. The parameter 'args' is filled
         * with arguments as defined in the drone wrapper's DRONE_WRAPPER(..) declaration and argument values are filled
         * in by the user on drone declaration to the platform at run-time
         * @param args a list of arguments to support the drone's initialisation
         */
        virtual void on_init(std::vector<std::string> args) = 0;

        /**
         * the last function to be called on the wrapper before the drone is removed from the platform. Do any cleanup
         * operations here.
         */
        virtual void on_deinit() = 0;

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
        virtual void on_motion_capture(const geometry_msgs::PoseStamped& msg) {};

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
         * @see set_drone_position
         */
        virtual void on_set_position(geometry_msgs::Vector3 pos, float yaw, float duration) = 0;

        /**
         * on_set_velocity is called whenever the drone should move with a desired velocity. This corresponds to a call to
         * set_drone_velocity in user_api.h
         * @param vel the desired velocity for the drone (xyz meters per second)
         * @param yawrate the desired yawrate for the drone (degrees per second)
         * @param duration TODO: what is duration in this case?
         */
        virtual void on_set_velocity(geometry_msgs::Vector3 vel, float yawrate, float duration) = 0;

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
        const std::string& get_tag();

        /**
         * returns the id of the rigidbody
         * @return the rigidbody's id
         */
        uint32_t get_id();

        /**
         * returns the current pose of the rigidbody
         * @return the rigidbody's current pose
         */
        const geometry_msgs::Pose& get_current_pose() const;

};


/** The DRONE_WRAPPER(..) macro is ordered as follows, the first parameter is the identifying tag of the drone and all following
 * parameters represent an argument to be passed into the on_init(..) function at drone startup. i.e. DRONE_WRAPPER(object, argA, argB) will
 * mean that the drone's tag is 'object' and under the on_init(std::vector<std::string> args) function 'args[0]' will hold the result of
 * argA when the drone is declared, and 'args[1]' will represent argB. The values of these args are given when the drone is declared onto
 * the drone platform through the 'add_drone' platform program.
 */
#define DRONE_WRAPPER(DroneName, ...) \
    DroneName : protected rigidbody { \
        public:\
            static std::string get_data_desc() { std::string desc = #__VA_ARGS__; desc.erase(remove_if(desc.begin(), desc.end(), ::isspace), desc.end()); return desc; } \
            DroneName(const std::string& tag, uint32_t id, std::vector<std::string> args) : rigidbody(tag, id) {this->on_init(std::move(args));} \
            ~DroneName() {this->on_deinit();} \
        private:

