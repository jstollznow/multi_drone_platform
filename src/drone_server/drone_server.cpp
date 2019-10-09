#include "ros/ros.h"
#include <vector>
#include <memory>

#include "../objects/rigidBody.h"
#include "wrappers.h"

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

        multi_drone_platform::mdpID addNewRigidbody(std::string pTag);
        void removeRigidbody(unsigned int pDroneID);

        bool getRigidbodyFromDroneID(const multi_drone_platform::mdpID pID, rigidBody* pReturnRigidbody);

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

    addNewRigidbody("cflie_00");
}

drone_server::~drone_server()
{
    /* cleanup all drone pointers in the rigidbody list */
    for (size_t i = 0; i < RigidBodyList.size(); i++) {
        if (RigidBodyList[i] != nullptr) {
            delete RigidBodyList[i];
        }
    }
    RigidBodyList.clear();
    printf("Shutting down drone server\n");
}

void drone_server::initialiseRigidbodiesFromVRPN()
{
    ROS_WARN("Initialising drones from vrpn is currently not supported, please add drones manually");
}

multi_drone_platform::mdpID drone_server::addNewRigidbody(std::string pTag)
{
    multi_drone_platform::mdpID ID;
    ID.name = pTag.c_str();
    ID.drone_id = RigidBodyList.size();
    RigidBodyList.push_back(mdp_wrappers::createNewRigidbody(pTag));
    // we link rigidbody to tag, but how do we link drone to rigidbody?
}

void drone_server::removeRigidbody(unsigned int pDroneID)
{
    /* if pDroneID is a valid index and the objectat that location is not null, delete */
    if (pDroneID < RigidBodyList.size()) {
        if (RigidBodyList[pDroneID] != nullptr) {
            delete RigidBodyList[pDroneID];
            /* and set to null */
            RigidBodyList[pDroneID] = nullptr;
        }
    }
}

bool drone_server::getRigidbodyFromDroneID(const multi_drone_platform::mdpID pID, rigidBody* pReturnRigidbody)
{
    if (pID.drone_id >= RigidBodyList.size()) return false;

    pReturnRigidbody = RigidBodyList[pID.drone_id];
    return (pReturnRigidbody == nullptr);
}

void drone_server::run()
{
    while (ros::ok()) {
        /* do all the ros callback event stuff */
        ros::spinOnce();

        /* call update on every valid rigidbody */
        for (size_t i = 0; i < RigidBodyList.size(); i++) {
            if (RigidBodyList[i] == nullptr) continue;

            RigidBodyList[i]->update(RigidBodyList);
        }
    }
    /* printf a newline to make terminal output better */
    printf("\n");
}

static std::map<std::string, int> APIMap = {
    {"VELOCITY", 0},    {"POSITION", 1},    {"TAKEOFF", 2},
    {"LAND", 3},        {"HOVER", 4},       {"EMERGENCY", 5},
    {"SET_HOME", 6},    {"GET_HOME", 7},    {"GOTO_HOME", 8}
};

void drone_server::APICallback(const multi_drone_platform::inputAPI::ConstPtr& msg)
{
    rigidBody* RB;
    if (!getRigidbodyFromDroneID(msg->drone_id, RB)) return;

    switch(APIMap[msg->msg_type]) {
        case 0: {   /* VELOCITY */
            RB->setDesVel(msg->data.posvel, msg->data.forward.z, msg->data.duration);
        } break;
        case 1: {   /* POSITION */
            RB->setDesPos(msg->data.posvel, msg->data.forward.z, msg->data.duration);
        } break;
        case 2: {   /* TAKEOFF */
            auto PosData = RB->getCurrPos();
            PosData.position.z = 1;
            RB->setDesPos(PosData.position, PosData.yaw, 0.0f);
        } break;
        case 3: {   /* LAND */
            // @TODO: we need a land command on the rigidbody to make use of the control loop (to soft land)
        } break;
        case 4: {   /* HOVER */
            auto PosData = RB->getCurrPos();
            RB->setDesPos(PosData.position, PosData.yaw, 0.0f);
        } break;
        case 5: {   /* EMERGENCY */
            // @TODO: need an emergency command on the rigidbody
        } break;
        case 6: {   /* SET_HOME */
            geometry_msgs::Vector3 Pos = msg->data.posvel;
            Pos.z = std::max(Pos.z, 1.0); // make sure the home position is above the ground
            RB->setHomePos(Pos);
        } break;
        case 8: {   /* GOTO_HOME */
            RB->setDesPos(RB->getHomePos(), 0.0f, 0.0f);
        } break;
        default: {
            ROS_ERROR_STREAM("The API command '" << msg->msg_type << "' is not a valid command for inputAPI");
        } break;
    }
}

bool drone_server::APIGetDataService(multi_drone_platform::droneFeedbackSRV::Request &Req, multi_drone_platform::droneFeedbackSRV::Response &Res)
{
    rigidBody* RB;
    if (!getRigidbodyFromDroneID(Req.drone_id, RB)) return false;

    switch(APIMap[Req.data_type]) {
        case 0: {   /* VELOCITY */
            auto RetVel = RB->getCurrVel();
            auto RetPos = RB->getCurrPos();
            Res.vec3 = RetVel.velocity;
            Res.forward.z = RetVel.yawRate;
            Res.forward.x = cos(-RetPos.yaw);
            Res.forward.y = sin(-RetPos.yaw);
            return true;
        } break;
        case 1: {   /* POSITION */
            auto RetVel = RB->getCurrVel();
            auto RetPos = RB->getCurrPos();
            Res.vec3 = RetPos.position;
            Res.forward.z = RetVel.yawRate;
            Res.forward.x = cos(-RetPos.yaw);
            Res.forward.y = sin(-RetPos.yaw);
            return true;
        } break;
        case 7: {   /* GET_HOME */
            Res.vec3 = RB->getHomePos();
            return true;
        } break;
        default: {
            ROS_ERROR_STREAM("The API command '" << Req.data_type << "' is not a valid command for dataFeedbackSRV");
        } break;
    }
    return false;
}

bool drone_server::APIListService(multi_drone_platform::rigidbodyListSRV::Request &Req, multi_drone_platform::rigidbodyListSRV::Response &Res)
{
    Res.drone_ids.clear();
    for (size_t i = 0; i < RigidBodyList.size(); i++) {
        if (RigidBodyList[i] == nullptr) continue;

        multi_drone_platform::mdpID ID;
        ID.drone_id = i;
        ID.name = "";
        Res.drone_ids.push_back(ID);
    }
    return true;
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    drone_server Server;
    Server.run();

    return 0;
}