#include "ros/ros.h"
#include <vector>
#include <memory>
#include <multi_drone_platform/add_drone.h>

#include "rigidbody.h"
#include "wrappers.h"
#include "../src/drone_server/drone_server_msg_translations.cpp"
#include "../icp_implementation/icp_impl.h"

#define LOOP_RATE_HZ 100
#define TIMING_UPDATE 5
#define POINT_SET_REG false

#define NODE_NAME "mdp_drone_server"
#define SRV_TOPIC "mdp_data_srv"
#define LIST_SRV_TOPIC "mdp_list_srv"
#define SUB_TOPIC "mdp"
#define EMERGENCY_TOPIC "mdp_emergency"
#define SHUTDOWN_PARAM "mdp/should_shut_down"
#define SESSION_PARAM "/mdp/session_directory"
#define ADD_DRONE_TOPIC "mdp/add_drone_srv"



class drone_server {
    private:
        /**
         * A vector containing all the rigidbodies on the drone server
         */
        std::vector<rigidbody*> rigidbodyList{};

        /**
         * The ROS time that the server started
         */
        ros::Time serverStartTime;

        /**
         * the drone server's node handle
         */
        ros::NodeHandle node;

        /**
         * Subscriber to read all incomming drone server and rigidbody API messages from user API programs
         */
        ros::Subscriber inputAPISub;

        /**
         * Subscriber dedicated to receive platform-wide emergency calls
         */
        ros::Subscriber emergencySub;

        ros::Publisher logPublisher;

        /**
         * ROS service servers for returning specific data to API programs and the declaration of drones at runtime
         */
        ros::ServiceServer listServer;
        ros::ServiceServer dataServer;
        ros::ServiceServer addDroneServer;

        /**
         * the loop rate that the server runs at
         */
        ros::Rate loopRate;

        /**
         * various time keeping variables
         */
        float desiredLoopRate = LOOP_RATE_HZ;
        float achievedLoopRate;
        float motionCaptureUpdateRate;
        float timeToUpdateDrones;
        float waitTime;

        /**
         * The entire drone server log for the session
         */
        std::string exportLog;

#if POINT_SET_REG
        icp_impl icpImplementation;
#endif /* POINT_SET_REG */

        /**
         * Adds a rigidbody to the drone server
         * @param pTag the tag to give the new drone (look at other documentation for details, "vflie_00")
         * @param args arguments to pass to the drone wrappers on_init function
         * @return a boolean if the addition succeeded or not
         */
        bool add_new_rigidbody(const std::string& pTag, std::vector<std::string> args);

        /**
         * removes a drone from the drone server
         * @param pDroneID a reference drone id
         */
        void remove_rigidbody(unsigned int pDroneID);

        /**
         * logs to the drone server's log
         * @param logType the type of log (INFO, WARN, etc.)
         * @param message a string message ("x is " + std::to_string(x))
         */
        void log(logger::log_type logType, std::string message);

        /**
         * returns the rigidbody object given a drone id
         * @param pID the id
         * @param pReturnRigidbody a pointer to a rigidbody that this function will return
         * @return boolean representing if the returned pointer is valid
         */
        bool get_rigidbody_from_drone_id(uint32_t pID, rigidbody* &pReturnRigidbody);

    public:
        drone_server();
        ~drone_server();

        /**
         * ROS callbacks for API and emergency
         * @param msg the message
         */
        void api_callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
        void emergency_callback(const std_msgs::Empty::ConstPtr& msg);

        /**
         * ROS Service server callbacks
         * @param req the request
         * @param res the response
         * @return valid
         */
        bool api_get_data_service(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res);
        bool api_list_service(tf2_msgs::FrameGraph::Request &req, tf2_msgs::FrameGraph::Response &res);
        bool add_drone_service(multi_drone_platform::add_drone::Request &req, multi_drone_platform::add_drone::Response &res);

        /**
         * main loop of the drone server
         */
        void run();

        /**
         * called on server shutdown
         */
        void shutdown();
};