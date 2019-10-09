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


typedef std_msgs::Header mdp_id;

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

        mdp_id addNewRigidbody(std::string pTag);
        void removeRigidbody(unsigned int pDroneID);

        bool getRigidbodyFromDroneID(mdp::id pID, rigidBody* pReturnRigidbody);

    public:
        drone_server();
        ~drone_server();

        void APICallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
        bool APIGetDataService(nav_msgs::GetPlan::Request &Req, nav_msgs::GetPlan::Response &Res);
        bool APIListService(nav_msgs::GetPlan::Request &Req, nav_msgs::GetPlan::Response &Res);

        void run();
};




drone_server::drone_server() : Node(), LoopRate(LOOP_RATE_HZ)
{
    ROS_INFO("Initialising drone server");
    InputAPISub = Node.subscribe<geometry_msgs::TransformStamped> (SUB_TOPIC, 10, &drone_server::APICallback, this);
    DataServer = Node.advertiseService(LIST_SRV_TOPIC, &drone_server::APIGetDataService, this);
    ListServer = Node.advertiseService(SRV_TOPIC, &drone_server::APIListService, this);
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

mdp_id drone_server::addNewRigidbody(std::string pTag)
{
    mdp_id ID;
    ID.frame_id = pTag.c_str();
    ID.seq = RigidBodyList.size();
    RigidBodyList.push_back(mdp_wrappers::createNewRigidbody(pTag));
    return ID;
    // we link rigidbody to tag, but how do we link drone to rigidbody? <drone_type>_<wrapper specific identifier> 'cflie_E7'
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

bool drone_server::getRigidbodyFromDroneID(mdp::id pID, rigidBody* pReturnRigidbody)
{
    if (pID.numeric_id() >= RigidBodyList.size()) return false;

    pReturnRigidbody = RigidBodyList[pID.numeric_id()];
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

void drone_server::APICallback(const geometry_msgs::TransformStamped::ConstPtr& input)
{
    mdp::input_msg msg((geometry_msgs::TransformStamped*)input.get());

    rigidBody* RB;
    if (!getRigidbodyFromDroneID(msg.drone_id(), RB)) return;

    switch(APIMap[msg.msg_type()]) {
        case 0: {   /* VELOCITY */
            RB->setDesVel(msg.posvel(), msg.yaw_rate(), msg.duration());
        } break;
        case 1: {   /* POSITION */
            RB->setDesPos(msg.posvel(), msg.yaw_rate(), msg.duration());
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
            geometry_msgs::Vector3 Pos = msg.posvel();
            Pos.z = std::max(Pos.z, 1.0); // make sure the home position is above the ground
            RB->setHomePos(Pos);
        } break;
        case 8: {   /* GOTO_HOME */
            RB->setDesPos(RB->getHomePos(), 0.0f, 0.0f);
        } break;
        default: {
            ROS_ERROR_STREAM("The API command '" << msg.msg_type() << "' is not a valid command for inputAPI");
        } break;
    }
}

bool drone_server::APIGetDataService(nav_msgs::GetPlan::Request &pReq, nav_msgs::GetPlan::Response &pRes)
{
    mdp::drone_feedback_srv_req Req(&pReq);
    mdp::drone_feedback_srv_res Res(&pRes);

    rigidBody* RB;
    if (!getRigidbodyFromDroneID(Req.drone_id, RB)) return false;

    switch(APIMap[Req.msg_type()]) {
        case 0: {   /* VELOCITY */
            auto RetVel = RB->getCurrVel();
            auto RetPos = RB->getCurrPos();
            Res.vec3().x = RetVel.velocity.x;
            Res.vec3().y = RetVel.velocity.y;
            Res.vec3().z = RetVel.velocity.z;
            Res.yaw_rate() = RetVel.yawRate;
            Res.forward_x() = cos(-RetPos.yaw);
            Res.forward_y() = sin(-RetPos.yaw);
            return true;
        } break;
        case 1: {   /* POSITION */
            auto RetVel = RB->getCurrVel();
            auto RetPos = RB->getCurrPos();
            Res.vec3().x = RetPos.position.x;
            Res.vec3().y = RetPos.position.y;
            Res.vec3().z = RetPos.position.z;
            Res.yaw_rate() = RetVel.yawRate;
            Res.forward_x() = cos(-RetPos.yaw);
            Res.forward_y() = sin(-RetPos.yaw);
            return true;
        } break;
        case 7: {   /* GET_HOME */
            geometry_msgs::Vector3 Pos = RB->getHomePos();
            Res.vec3().x = Pos.x;
            Res.vec3().y = Pos.y;
            Res.vec3().z = Pos.z;
            return true;
        } break;
        default: {
            ROS_ERROR_STREAM("The API command '" << Req.msg_type() << "' is not a valid command for dataFeedbackSRV");
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