#include "ros/ros.h"
#include <vector>
#include <memory>

#include "drone_server_msg_translations.cpp"

#include "../objects/rigidBody.h"
#include "wrappers.h"

#define LOOP_RATE_HZ 100

#define NODE_NAME "mdp_drone_server"
#define SRV_TOPIC "mdp_api_data_srv"
#define LIST_SRV_TOPIC "mdp_api_list_srv"
#define SUB_TOPIC "mdp_api"
#define EMERGENCY_TOPIC "mdp_api_emergency"


struct mdp_id{
    std::string name = "";
    uint32_t numeric_id = 0;
    bool isValid() const {return (name.length() != 0);}
};

class drone_server
{
    private:
        std::vector<rigidBody*> RigidBodyList;

        ros::NodeHandle Node;
        ros::Subscriber InputAPISub;
        ros::Subscriber EmergencySub;
        ros::ServiceServer ListServer;
        ros::ServiceServer DataServer;

        ros::Rate LoopRate;
        float DesiredLoopRate = LOOP_RATE_HZ;
        float AchievedLoopRate;
        float MotionCaptureUpdateRate;
        float TimeToUpdateDrones;
        float WaitTime;

        void initialiseRigidbodiesFromVRPN();

        mdp_id addNewRigidbody(std::string pTag);
        void removeRigidbody(unsigned int pDroneID);

        bool getRigidbodyFromDroneID(uint32_t pID, rigidBody* &pReturnRigidbody);
        void setVelocityOnDrone(rigidBody* RB, mdp::input_msg& msg);
        void setPositionOnDrone(rigidBody* RB, mdp::input_msg& msg);

    public:
        drone_server();
        ~drone_server();

        void APICallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
        void NewAPICallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
        void EmergencyCallback(const std_msgs::Empty::ConstPtr& msg);
        bool APIGetDataService(nav_msgs::GetPlan::Request &Req, nav_msgs::GetPlan::Response &Res);
        bool APIListService(tf2_msgs::FrameGraph::Request &Req, tf2_msgs::FrameGraph::Response &Res);

        void run();
};