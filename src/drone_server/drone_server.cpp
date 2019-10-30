#include "drone_server.h"
#include "multi_drone_platform/apiUpdate.h"

// constructor
drone_server::drone_server() : Node(), LoopRate(LOOP_RATE_HZ)
{
    ROS_INFO("Initialising drone server");
    InputAPISub = Node.subscribe<geometry_msgs::TransformStamped> (SUB_TOPIC, 10, &drone_server::APICallback, this);
    EmergencySub = Node.subscribe<std_msgs::Empty> (EMERGENCY_TOPIC, 10, &drone_server::EmergencyCallback, this);
    DataServer = Node.advertiseService(SRV_TOPIC, &drone_server::APIGetDataService, this);
    ListServer = Node.advertiseService(LIST_SRV_TOPIC, &drone_server::APIListService, this);
    std::string droneName1;
    std::string droneName2;
    // if (Node.hasParam("cflie_test"))
    // {
    //     Node.getParam("cflie_test", droneName1);
    //     ROS_INFO("Adding %s", droneName1.c_str());
    //     addNewRigidbody(droneName1);
    // }
    // ROS_INFO("Adding next drone");
    // if (Node.hasParam("cflie_test1"))
    // {
    //     Node.getParam("cflie_test1", droneName2);
    //     ROS_INFO("Adding %s", droneName2.c_str());
    //     addNewRigidbody(droneName2);
    // }
    addNewRigidbody("object_00");

    addNewRigidbody("cflie_00");
    // ros::Duration d(2.0);
    // d.sleep();
    // addNewRigidbody("cflie_E7");
}

// deconstructor
drone_server::~drone_server()
{
    /* cleanup all drone pointers in the rigidbody list */
    for (size_t i = 0; i < RigidBodyList.size(); i++) {
        removeRigidbody(i);
    }
    RigidBodyList.clear();
    printf("Shutting down drone server\n");
}

// @TODO: implementation task on Trello
void drone_server::initialiseRigidbodiesFromVRPN()
{
    ROS_WARN("Initialising drones from vrpn is currently not supported, please add drones manually");
}


mdp_id drone_server::addNewRigidbody(std::string pTag)
{
    mdp_id ID;
    ID.name = "";
    ID.numeric_id = 0;
    rigidBody* RB;
    if (mdp_wrappers::createNewRigidbody(pTag, RB)) {
        /* update drone state on param server */

        RigidBodyList.push_back(RB);
        ID.name = pTag.c_str();
        ID.numeric_id = RigidBodyList.size()-1;
        RB->setID(ID.numeric_id);
        // setup VRPN Callback Queue
        RB->mySpin.start();

        ROS_INFO_STREAM("Successfully added drone with the tag: " << pTag);
    } else {
        ROS_ERROR_STREAM("Unable to add drone with tag: '" << pTag << "', check if drone type naming is correct.");
    }
    return ID;
}

void drone_server::removeRigidbody(unsigned int pDroneID)
{
    /* if pDroneID is a valid index and the object at that location is not null, delete */
    if (pDroneID < RigidBodyList.size()) {
        if (RigidBodyList[pDroneID] != nullptr) {
            delete RigidBodyList[pDroneID];
            /* and set to null */
            RigidBodyList[pDroneID] = nullptr;
            
            /* update drone state on param server */
            // @FIX: shoudn't we just remove the parameter?
            // Node.param.deleteParam?
            Node.setParam("mdp/drone_" + std::to_string(pDroneID) + "/state", "DELETED");
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
    int timingPrint = 0;
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
                // ROS_WARN("Looprate false");
            }
        }
        WaitTimeEnd = ros::Time::now();

        FrameEnd = ros::Time::now();

        /* record timing information */
        AchievedLoopRate += (1.0 / (FrameEnd.toSec() - FrameStart.toSec()));
        WaitTime += (WaitTimeEnd.toSec() - WaitTimeStart.toSec());
        TimeToUpdateDrones += (RigidBodyEnd.toSec() - RigidBodyStart.toSec());


        if (timingPrint >= TIMING_UPDATE*LOOP_RATE_HZ) // 5 seconds
        {
            float avgLoopRate = AchievedLoopRate/timingPrint;
            float avgWaitTime = WaitTime/timingPrint;
            float avgDroneUpdate = TimeToUpdateDrones/timingPrint;
            timingPrint = 0;
            ROS_INFO("Avg. Loop Info--");
            ROS_INFO("Actual [Hz]: %.2f, Wait [s]: %.4f, Drones [s]: %.4f",avgLoopRate, avgWaitTime, avgDroneUpdate);
            AchievedLoopRate = 0.0f;
            WaitTime = 0.0f;
            TimeToUpdateDrones = 0.0f;
        }
        timingPrint++;
    }
    // for formatting
    printf("\n");
}

// static std::map<std::string, int> APIMap = {
//     {"VELOCITY", 0},    {"POSITION", 1},    {"TAKEOFF", 2},
//     {"LAND", 3},        {"HOVER", 4},       {"EMERGENCY", 5},
//     {"SET_HOME", 6},    {"GET_HOME", 7},    {"GOTO_HOME", 8},
//     {"ORIENTATION", 9}, {"TIME", 10},       {"DRONE_SERVER_FREQ", 11}
// };

void drone_server::EmergencyCallback(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_ERROR("EMERGENCY CALLED ON DRONE SERVER");
    for (size_t i = 0; i < RigidBodyList.size(); i++) {
        if (RigidBodyList[i] != nullptr) {
            // is there any reason why this is called twice?
            RigidBodyList[i]->emergency();
        }
    }
    for (size_t i = 0; i < RigidBodyList.size(); i++) {
        if (RigidBodyList[i] != nullptr) {
            // is there any reason why this is called twice?
            RigidBodyList[i]->emergency();
        }
    }
}

std::array<bool, 2> dencoded_relative(double pEncoded)
{
    uint32_t pEncodedInt = (uint32_t)pEncoded;
    std::array<bool, 2> ret_arr;
    ret_arr[0] = (pEncodedInt & 0x00000001) > 0;
    ret_arr[1] = (pEncodedInt & 0x00000002) > 0;
    return ret_arr;
}

void drone_server::APICallback(const geometry_msgs::TransformStamped::ConstPtr& input)
{
    mdp::input_msg Input((geometry_msgs::TransformStamped*)input.get());
    multi_drone_platform::apiUpdate msg;

    ROS_INFO("drone server recieved %s", Input.msg_type().c_str());

    rigidBody* RB;
    if (!getRigidbodyFromDroneID(Input.drone_id().numeric_id(), RB)) {
        return;
    }

    msg.msg_type = Input.msg_type();
    
    msg.posvel.x   = Input.posvel().x;
    msg.posvel.y   = Input.posvel().y;
    msg.posvel.z   = Input.posvel().z;

    msg.yawVal = Input.yaw();
    ROS_INFO("duration is %f", Input.duration());
    msg.duration = Input.duration();
    
    auto RelativeArr = dencoded_relative(Input.relative());
    msg.relative = RelativeArr[0];
    msg.constHeight = RelativeArr[1];

    RB->ApiPublisher.publish(msg);    
    // send the message off to the relevant rigidbody
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
        // RB SIDE
            auto RetVel = RB->getCurrVel();
            auto RetPos = RB->getCurrPos();
            // can get a ros::Time back from these messages now
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
        // RB SIDE
            auto RetVel = RB->getCurrVel();
            auto RetPos = RB->getCurrPos();
            // can get a ros::Time back from these messages now
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
        // RB SIDE
            geometry_msgs::Vector3 Pos = RB->getHomePos();
            Res.vec3().x = Pos.x;
            Res.vec3().y = Pos.y;
            Res.vec3().z = Pos.z;
            ROS_INFO_STREAM("Server completed get data service of type: " << Req.msg_type());
            return true;
        } break;
        case 9: {   /* ORIENTATION */
        // RB SIDE
            auto RetVel = RB->getCurrVel();
            auto RetPos = RB->getCurrPos();
            ROS_INFO_STREAM("Server completed get data service of type: " << Req.msg_type());
            return true;
        } break;
        case 10: {  /* TIME */
        // SERVER SIDE
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
        // printf("adding RB to list: %d, %s\n", i, RigidBodyList[i]->getName().c_str());
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