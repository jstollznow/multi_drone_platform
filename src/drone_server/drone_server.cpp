#include "ros/ros.h"
#include <vector>
#include <memory>

#include "../objects/rigidBody.h"

#include "multi_drone_platform/inputAPI.h"
#include "multi_drone_platform/droneFeedbackSRV.h"
#include "multi_drone_platform/rigidbodyListSRV.h"
#include "geometry_msgs/PoseStamped.h"

#define LOOP_RATE_HZ 100

#define NODE_NAME "mdp_drone_server"
#define SRV_TOPIC "mdp_api_srv"
#define LIST_SRV_TOPIC "mdp_api_list_srv"
#define SUB_TOPIC "mdp_api"



class drone_server
{
    private:
        std::vector<rigidBody*> RigidBodyList;

        ros::NodeHandle Node;
        ros::Subscriber InputAPISub;
        ros::ServiceServer ListServer;
        ros::ServiceServer DataServer;

        ros::Rate LoopRate;

        void initialiseRigidbodiesFromVRPN();

    public:
        drone_server();
        ~drone_server();

        void APICallback(const multi_drone_platform::inputAPI::ConstPtr& msg);
        bool APIGetDataService(multi_drone_platform::droneFeedbackSRV::Request &Req, multi_drone_platform::droneFeedbackSRV::Response &Res);
        bool APIListService(multi_drone_platform::rigidbodyListSRV::Request &Req, multi_drone_platform::rigidbodyListSRV::Response &Res);

        void run();
};




drone_server::drone_server() : Node(), LoopRate(LOOP_RATE_HZ)
{
    ROS_INFO("Initialising drone server");
    InputAPISub = Node.subscribe<multi_drone_platform::inputAPI> (SUB_TOPIC, 10, &drone_server::APICallback, this);
    DataServer = Node.advertiseService(LIST_SRV_TOPIC, &drone_server::APIGetDataService, this);
    ListServer = Node.advertiseService(SRV_TOPIC, &drone_server::APIListService, this);
}

drone_server::~drone_server()
{
    ROS_INFO("Shutting down drone server");
    for (size_t i = 0; i < RigidBodyList.size(); i++) {
        delete RigidBodyList[i];
    }
    RigidBodyList.clear();
}

void drone_server::initialiseRigidbodiesFromVRPN()
{

}

void drone_server::run()
{
    while (ros::ok) {
        ros::spinOnce();

        for (size_t i = 0; i < RigidBodyList.size(); i++) {
            RigidBodyList[i]->update(RigidBodyList);
        }
    }
}

void drone_server::APICallback(const multi_drone_platform::inputAPI::ConstPtr& msg)
{

}

bool drone_server::APIGetDataService(multi_drone_platform::droneFeedbackSRV::Request &Req, multi_drone_platform::droneFeedbackSRV::Response &Res)
{
    return true;
}

bool drone_server::APIListService(multi_drone_platform::rigidbodyListSRV::Request &Req, multi_drone_platform::rigidbodyListSRV::Response &Res)
{
    return true;
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    drone_server Server;
    Server.run();

    return 0;
}