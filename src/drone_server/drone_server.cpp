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

    public:
        drone_server();
        ~drone_server();

        void APICallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
        bool APIGetDataService(nav_msgs::GetPlan::Request &Req, nav_msgs::GetPlan::Response &Res);
        bool APIListService(tf2_msgs::FrameGraph::Request &Req, tf2_msgs::FrameGraph::Response &Res);

        void run();
};




drone_server::drone_server() : Node(), LoopRate(LOOP_RATE_HZ)
{
    ROS_INFO("Initialising drone server");
    InputAPISub = Node.subscribe<geometry_msgs::TransformStamped> (SUB_TOPIC, 10, &drone_server::APICallback, this);
    DataServer = Node.advertiseService(LIST_SRV_TOPIC, &drone_server::APIGetDataService, this);
    ListServer = Node.advertiseService(SRV_TOPIC, &drone_server::APIListService, this);
    std::string droneName;
    ROS_INFO("is this your drone? %s", droneName.c_str());
    if (Node.hasParam("cflie_test"))
    {
        Node.getParam("cflie_test", droneName);
        addNewRigidbody(droneName);
    }

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
    ID.name = pTag.c_str();
    ID.numeric_id = RigidBodyList.size();
    rigidBody* RB;
    if (mdp_wrappers::createNewRigidbody(pTag, RB)) {
        RigidBodyList.push_back(RB);
        ID.name = pTag.c_str();
        ID.numeric_id = RigidBodyList.size();
        ROS_INFO_STREAM("Successfully added drone with the tag: " << pTag);
    } else {
        ID.name = "";
        ID.numeric_id = 0;
        ROS_ERROR_STREAM("Unable to add drone with tag: '" << pTag << "', check if drone type naming is correct.");
    }
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

bool drone_server::getRigidbodyFromDroneID(uint32_t pID, rigidBody* &pReturnRigidbody)
{
    if (pID >= RigidBodyList.size()) {
        ROS_WARN("supplied ID is greater than size of rigidbody list: %d >= %d", pID, RigidBodyList.size());
        return false;
    }

    pReturnRigidbody = RigidBodyList[pID];
    return (pReturnRigidbody != nullptr);
}

void drone_server::run()
{
    ros::Time FrameStart, FrameEnd, RigidBodyStart, RigidBodyEnd, WaitTimeStart, WaitTimeEnd;
    while (ros::ok()) {
        FrameStart = ros::Time::now();
        /* do all the ros callback event stuff */
        ros::spinOnce();

        /* call update on every valid rigidbody */
        RigidBodyStart = ros::Time::now();
        for (size_t i = 0; i < RigidBodyList.size(); i++) {
            if (RigidBodyList[i] == nullptr) continue;

            RigidBodyList[i]->update(RigidBodyList);
        }
        RigidBodyEnd = ros::Time::now();
        
        /* wait remainder of looprate */
        WaitTimeStart = ros::Time::now();
        if (DesiredLoopRate > 0.0) {
            if (!LoopRate.sleep()) {
                ROS_WARN("Looprate false");
            }
        }
        WaitTimeEnd = ros::Time::now();

        FrameEnd = ros::Time::now();

        /* record timing information */
        AchievedLoopRate = (1.0 / (FrameEnd.toSec() - FrameStart.toSec()));
        WaitTime = (WaitTimeEnd.toSec() - WaitTimeStart.toSec());
        TimeToUpdateDrones = (RigidBodyEnd.toSec() - RigidBodyStart.toSec());
    }
    /* printf a newline to make terminal output better */
    printf("\n");
}

static std::map<std::string, int> APIMap = {
    {"VELOCITY", 0},    {"POSITION", 1},    {"TAKEOFF", 2},
    {"LAND", 3},        {"HOVER", 4},       {"EMERGENCY", 5},
    {"SET_HOME", 6},    {"GET_HOME", 7},    {"GOTO_HOME", 8},
    {"ORIENTATION", 9}, {"TIME", 10},       {"DRONE_SERVER_FREQ", 11}
};

void drone_server::APICallback(const geometry_msgs::TransformStamped::ConstPtr& input)
{
    mdp::input_msg msg((geometry_msgs::TransformStamped*)input.get());

    ROS_INFO_STREAM("Server recieved set data call of type: " << msg.msg_type());

    rigidBody* RB;
    getRigidbodyFromDroneID(msg.drone_id().numeric_id(), RB);

    switch(APIMap[msg.msg_type()]) {
        case 0: {   /* VELOCITY */
            if (RB == nullptr) return;
            RB->setDesVel(msg.posvel(), msg.yaw_rate(), msg.duration());
        } break;
        case 1: {   /* POSITION */
            if (RB == nullptr) return;
            RB->setDesPos(msg.posvel(), msg.yaw_rate(), msg.duration());
        } break;
        case 2: {   /* TAKEOFF */
            if (RB == nullptr) return;
            auto PosData = RB->getCurrPos();
            PosData.position.z = 1;
            RB->setDesPos(PosData.position, PosData.yaw, 0.0f);
        } break;
        case 3: {   /* LAND */
            if (RB == nullptr) return;
            // @TODO: we need a land command on the rigidbody to make use of the control loop (to soft land)
        } break;
        case 4: {   /* HOVER */
            if (RB == nullptr) return;
            auto PosData = RB->getCurrPos();
            RB->setDesPos(PosData.position, PosData.yaw, 0.0f);
        } break;
        case 5: {   /* EMERGENCY */
            if (RB == nullptr) return;
            // @TODO: need an emergency command on the rigidbody
        } break;
        case 6: {   /* SET_HOME */
            if (RB == nullptr) return;
            geometry_msgs::Vector3 Pos = msg.posvel();
            Pos.z = std::max(Pos.z, 1.0); // make sure the home position is above the ground
            RB->setHomePos(Pos);
        } break;
        case 8: {   /* GOTO_HOME */
            if (RB == nullptr) return;
            RB->setDesPos(RB->getHomePos(), 0.0f, 0.0f);
        } break;
        case 11: {  /* DRONE_SERVER_FREQ */
            if (msg.posvel().x > 0.0) {
                this->LoopRate = ros::Rate(msg.posvel().x);
            }
            DesiredLoopRate = msg.posvel().x;
        } break;
        default: {
            ROS_ERROR_STREAM("The API command '" << msg.msg_type() << "' is not a valid command for inputAPI");
        } break;
    }
    ROS_INFO_STREAM("Server completed the set data call of type: " << msg.msg_type());
}

bool drone_server::APIGetDataService(nav_msgs::GetPlan::Request &pReq, nav_msgs::GetPlan::Response &pRes)
{
    mdp::drone_feedback_srv_req Req(&pReq);
    mdp::drone_feedback_srv_res Res(&pRes);
    ROS_INFO_STREAM("Server recieved get data service of type: " << Req.msg_type());
    
    rigidBody* RB;
    if (!getRigidbodyFromDroneID(Req.drone_id().numeric_id(), RB)) return false;

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
            ROS_INFO_STREAM("Server completed get data service of type: " << Req.msg_type());
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
            ROS_INFO_STREAM("Server completed get data service of type: " << Req.msg_type());
            return true;
        } break;
        case 7: {   /* GET_HOME */
            geometry_msgs::Vector3 Pos = RB->getHomePos();
            Res.vec3().x = Pos.x;
            Res.vec3().y = Pos.y;
            Res.vec3().z = Pos.z;
            ROS_INFO_STREAM("Server completed get data service of type: " << Req.msg_type());
            return true;
        } break;
        case 9: {   /* ORIENTATION */
            auto RetVel = RB->getCurrVel();
            auto RetPos = RB->getCurrPos();
            ROS_INFO_STREAM("Server completed get data service of type: " << Req.msg_type());
            return true;
        } break;
        case 10: {  /* TIME */
            Res.vec3().x = DesiredLoopRate;
            Res.vec3().y = AchievedLoopRate;
            Res.vec3().z = MotionCaptureUpdateRate;
            Res.forward_x() = TimeToUpdateDrones;
            Res.forward_y() = WaitTime;
            ROS_INFO_STREAM("Server completed get data service of type: " << Req.msg_type());
            return true;
        } break;
        default: {
            ROS_ERROR_STREAM("The API command '" << Req.msg_type() << "' is not a valid command for dataFeedbackSRV");
        } break;
    }
    ROS_WARN_STREAM("Server failed get data service of type: " << Req.msg_type());
    return false;
}

bool drone_server::APIListService(tf2_msgs::FrameGraph::Request &Req, tf2_msgs::FrameGraph::Response &Res)
{
    /* encoding for the list service is done here without a helper class */
    /* check API functions documentation for clarity */
    Res.frame_yaml = "";
    for (size_t i = 0; i < RigidBodyList.size(); i++) {
        if (RigidBodyList[i] == nullptr) continue;

        Res.frame_yaml += std::to_string(i) + ":" + RigidBodyList[i]->getName() + " ";
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