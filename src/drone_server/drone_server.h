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

#define NODE_NAME "mdp_drone_server"
#define SRV_TOPIC "mdp_data_srv"
#define LIST_SRV_TOPIC "mdp_list_srv"
#define SUB_TOPIC "mdp"
#define EMERGENCY_TOPIC "mdp_emergency"
#define SHUTDOWN_PARAM "mdp/should_shut_down"
#define ADD_DRONE_TOPIC "mdp/add_drone_srv"


class drone_server {
    private:
        std::vector<rigidbody*> rigidbodyList{};

        ros::NodeHandle node;
        ros::Subscriber inputAPISub;
        ros::Subscriber emergencySub;
        ros::Publisher logPublisher;
        ros::ServiceServer listServer;
        ros::ServiceServer dataServer;
        ros::ServiceServer addDroneServer;

        ros::Rate loopRate;
        float desiredLoopRate = LOOP_RATE_HZ;
        float achievedLoopRate;
        float motionCaptureUpdateRate;
        float timeToUpdateDrones;
        float waitTime;

        // @TODO: place this under a define so that we can enable or disable this feature
        icp_impl icpImplementation;

        void init_rigidbodies_from_VRPN();

        bool add_new_rigidbody(const std::string& pTag, std::vector<std::string> args);
        void remove_rigidbody(unsigned int pDroneID);

        void log(logger::log_type logType, std::string message);

        bool get_rigidbody_from_drone_id(uint32_t pID, rigidbody* &pReturnRigidbody);

    public:
        drone_server();
        ~drone_server();

        void api_callback(const geometry_msgs::TransformStamped::ConstPtr& msg);
        void emergency_callback(const std_msgs::Empty::ConstPtr& msg);
        bool api_get_data_service(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res);
        bool api_list_service(tf2_msgs::FrameGraph::Request &req, tf2_msgs::FrameGraph::Response &res);
        bool add_drone_service(multi_drone_platform::add_drone::Request &req, multi_drone_platform::add_drone::Response &res);

        void run();
        void shutdown();
};