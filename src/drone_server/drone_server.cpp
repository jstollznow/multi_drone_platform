#include "ros/ros.h"
#include <vector>
#include <memory>

#include "../objects/rigidBody.h"

#include "multi_drone_platform/inputAPI.h"
#include "multi_drone_platform/movementFeedbackSRV.h"
#include "geometry_msgs/PoseStamped.h"

#define LOOP_RATE_HZ 100

#define NODE_NAME  "mdp_drone_server"
#define SRV_TOPIC "mdp_api_srv"
#define SUB_TOPIC "mdp_api"


void API_input(const multi_drone_platform::inputAPI& Msg)
{
    ROS_INFO("Recieved command from api");
    // ROS_INFO("Msg: %s %d %f %f %f %f", Msg.msg_type.c_str(), Msg.drone_id, Msg.movement.vec3.x,Msg.movement.vec3.y,Msg.movement.vec3.z);
}

bool API_get_data_srv(multi_drone_platform::movementFeedbackSRV::Request &Req, multi_drone_platform::movementFeedbackSRV::Response &Res)
{
    if (strcmp(Req.data_type.c_str(), "VELOCITY") == 0) {
        Res.vec3.x = -1.0f;
        Res.vec3.y = -2.0f;
        Res.vec3.z = -3.0f;
        Res.yaw    = -4.0f;
        return true;
    } else if (strcmp(Req.data_type.c_str(), "POSITION") == 0) {
        Res.vec3.x = 1.0f;
        Res.vec3.y = 2.0f;
        Res.vec3.z = 3.0f;
        Res.yaw    = 4.0f;
        return true;
    }
    return false;
}


class drone_server
{
    private:
        std::vector<rigidBody*> RigidBodyList;

        ros::NodeHandle Node;
        ros::Subscriber InputAPISub;
        ros::ServiceServer ListServer;
        ros::ServiceServer DataServer;

        void initialiseRigidbodiesFromVRPN();

    public:
        drone_server();
        ~drone_server();

        void run();
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    ros::NodeHandle Node;

    ros::ServiceServer SrvServer = Node.advertiseService(SRV_TOPIC, API_get_data_srv);
    ros::Subscriber Sub = Node.subscribe(SUB_TOPIC, 10, &API_input);

    ros::Rate LoopRate(LOOP_RATE_HZ);

    while (ros::ok())
    {
        ros::spinOnce();
        LoopRate.sleep();
    }

    return 0;
}