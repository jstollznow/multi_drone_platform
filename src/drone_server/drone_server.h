#include "ros/ros.h"
#include <vector>
#include <memory>

#include "rigidbody.h"
#include "wrappers.h"
#include "../src/drone_server/drone_server_msg_translations.cpp"

#define LOOP_RATE_HZ 100
#define TIMING_UPDATE 5

#define NODE_NAME "mdp_drone_server"
#define SRV_TOPIC "mdp_api_data_srv"
#define LIST_SRV_TOPIC "mdp_api_list_srv"
#define SUB_TOPIC "mdp_api"
#define EMERGENCY_TOPIC "mdp_api_emergency"


struct mdp_id {
    std::string name = "";
    uint32_t numeric_id = 0;
};

class drone_server {
    private:
        std::vector<rigidbody*> rigidbodyList;

        ros::NodeHandle node;
        ros::Subscriber inputAPISub;
        ros::Subscriber emergencySub;
        ros::Publisher logPublisher;
        ros::ServiceServer listServer;
        ros::ServiceServer dataServer;

        ros::Rate loopRate;
        float desiredLoopRate = LOOP_RATE_HZ;
        float achievedLoopRate;
        float motionCaptureUpdateRate;
        float timeToUpdateDrones;
        float waitTime;

        void init_rigidbodies_from_VRPN();

        mdp_id add_new_rigidbody(std::string pTag);
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

        void run();
        void shutdown();
};