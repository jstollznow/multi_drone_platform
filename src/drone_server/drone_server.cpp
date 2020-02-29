#include "drone_server.h"
#include "multi_drone_platform/api_update.h"


drone_server::drone_server() : node(), loopRate(LOOP_RATE_HZ) {
    ROS_INFO("Initialising drone server1");
    inputAPISub = node.subscribe<geometry_msgs::TransformStamped> (SUB_TOPIC, 2, &drone_server::api_callback, this);
    emergencySub = node.subscribe<std_msgs::Empty> (EMERGENCY_TOPIC, 10, &drone_server::emergency_callback, this);
    dataServer = node.advertiseService(SRV_TOPIC, &drone_server::api_get_data_service, this);
    listServer = node.advertiseService(LIST_SRV_TOPIC, &drone_server::api_list_service, this);
    std::string droneName1;
    std::string droneName2;
    // if (Node.hasParam("cflie_test1"))
    // {
    //     Node.getParam("cflie_test1", droneName2);
    //     ROS_INFO("Adding %s", droneName2.c_str());
    //     addNewRigidbody(droneName2);
    // }
    add_new_rigidbody("vflie_00");
    add_new_rigidbody("vflie_01");
}

drone_server::~drone_server() {
    this->shutdown();
}

void drone_server::shutdown() {
    for (size_t i = 0; i < rigidbodyList.size(); i++) {
        remove_rigidbody(i);
    }
    rigidbodyList.clear();
    printf("Shutting down drone server\n");
}

// @TODO: implementation task on Trello
void drone_server::init_rigidbodies_from_VRPN() {
    ROS_WARN("Initialising drones from vrpn is currently not supported, please add drones manually");
}


mdp_id drone_server::add_new_rigidbody(std::string pTag) {
    mdp_id ID;
    ID.name = "";
    ID.numeric_id = rigidbodyList.size();
    rigidbody* RB;
    if (mdp_wrappers::create_new_rigidbody(pTag, ID.numeric_id, RB)) {
        /* update drone state on param server */

        rigidbodyList.push_back(RB);
        ID.name = pTag.c_str();
        
        RB->mySpin.start();

        ROS_INFO_STREAM("Successfully added drone with the tag: " << pTag);
    } else {
        ROS_ERROR_STREAM("Unable to add drone with tag: '" << pTag << "', check if drone type naming is correct.");
    }
    return ID;
}

void drone_server::remove_rigidbody(unsigned int pDroneID) {
    /* if pDroneID is a valid index and the object at that location is not null, delete */
    if (pDroneID < rigidbodyList.size()) {
        if (rigidbodyList[pDroneID] != nullptr) {
            delete rigidbodyList[pDroneID];
            /* and set to null */
            rigidbodyList[pDroneID] = nullptr;
            
            /* update drone state on param server */
            // @FIX: shoudn't we just remove the parameter?
            // Node.param.deleteParam?
            node.deleteParam("mdp/drone_" + std::to_string(pDroneID) + "/state");
        }
    }
}

bool drone_server::get_rigidbody_from_drone_id(uint32_t pID, rigidbody*& pReturnRigidbody) {
    if (pID >= rigidbodyList.size()) {
        ROS_WARN("supplied ID is greater than size of rigidbody list: %d >= %d", pID, rigidbodyList.size());
        return false;
    }

    pReturnRigidbody = rigidbodyList[pID];
    return (pReturnRigidbody != nullptr);
}

void drone_server::run() {
    ros::Time frameStart, frameEnd, rigidbodyStart, rigidbodyEnd, waitTimeStart, waitTimeEnd;
    int timingPrint = 0;
    while (ros::ok()) {
        frameStart = ros::Time::now();
        /* do all the ros callback event stuff */
        ros::spinOnce();

        /* call update on every valid rigidbody */
        rigidbodyStart = ros::Time::now();
        for (size_t i = 0; i < rigidbodyList.size(); i++) {
            if (rigidbodyList[i] == nullptr) continue;

            rigidbodyList[i]->update(rigidbodyList);

        }
        rigidbodyEnd = ros::Time::now();
        
        /* wait remainder of looprate */
        waitTimeStart = ros::Time::now();
        if (desiredLoopRate > 0.0) {
            if (!loopRate.sleep()) {
                // ROS_WARN("Looprate false");
            }
        }
        waitTimeEnd = ros::Time::now();

        frameEnd = ros::Time::now();

        /* record timing information */
        achievedLoopRate += (1.0 / (frameEnd.toSec() - frameStart.toSec()));
        waitTime += (waitTimeEnd.toSec() - waitTimeStart.toSec());
        timeToUpdateDrones += (rigidbodyEnd.toSec() - rigidbodyStart.toSec());


        if (timingPrint >= TIMING_UPDATE*LOOP_RATE_HZ) // 5 seconds
        {
            float avgLoopRate = achievedLoopRate/timingPrint;
            float avgWaitTime = waitTime/timingPrint;
            float avgDroneUpdate = timeToUpdateDrones/timingPrint;
            timingPrint = 0;
            ROS_INFO("Avg. Loop Info--");
            ROS_INFO("Actual [Hz]: %.2f, Wait [s]: %.4f, Drones [s]: %.4f",
            avgLoopRate, avgWaitTime, avgDroneUpdate);

            achievedLoopRate = 0.0f;
            waitTime = 0.0f;
            timeToUpdateDrones = 0.0f;
        }
        timingPrint++;
    }
    ros::Duration d(2.0);
    d.sleep();
    std::cout<<std::endl;
}

void drone_server::emergency_callback(const std_msgs::Empty::ConstPtr& msg) {
    // twice for assurance
    ROS_ERROR("EMERGENCY CALLED ON DRONE SERVER");
    for (size_t i = 0; i < rigidbodyList.size(); i++) {
        if (rigidbodyList[i] != nullptr) {
            rigidbodyList[i]->emergency();
        }
    }
    for (size_t i = 0; i < rigidbodyList.size(); i++) {
        if (rigidbodyList[i] != nullptr) {
            rigidbodyList[i]->emergency();
        }
    }
}

std::array<bool, 2> dencoded_relative(double pEncoded) {
    uint32_t pEncodedInt = (uint32_t)pEncoded;
    std::array<bool, 2> ret_arr;
    ret_arr[0] = (pEncodedInt & 0x00000001) > 0;
    ret_arr[1] = (pEncodedInt & 0x00000002) > 0;
    return ret_arr;
}

void drone_server::api_callback(const geometry_msgs::TransformStamped::ConstPtr& input) {
    mdp::input_msg inputMsg((geometry_msgs::TransformStamped*)input.get());
    multi_drone_platform::api_update msg;

    // ROS_INFO("drone server recieved %s", Input.msg_type().c_str());

    rigidbody* RB;
    if (!get_rigidbody_from_drone_id(inputMsg.drone_id().numeric_id(), RB)) {
        return;
    }

    msg.msgType = inputMsg.msg_type();
    
    msg.posVel.x   = inputMsg.pos_vel().x;
    msg.posVel.y   = inputMsg.pos_vel().y;
    msg.posVel.z   = inputMsg.pos_vel().z;

    msg.yawVal = inputMsg.yaw();
    msg.duration = inputMsg.duration();
    
    auto relativeArr = dencoded_relative(inputMsg.relative());
    msg.relativeXY = relativeArr[0];
    msg.relativeZ  = relativeArr[1];

    RB->apiPublisher.publish(msg);    
    // send the message off to the relevant rigidbody
}

bool drone_server::api_get_data_service(nav_msgs::GetPlan::Request &pReq, nav_msgs::GetPlan::Response &pRes) {
    mdp::drone_feedback_srv_req req(&pReq);
    mdp::drone_feedback_srv_res res(&pRes);
    ROS_INFO_STREAM("Server recieved get data service of type: " << req.msgType());
    
    rigidbody* RB;
    if (!get_rigidbody_from_drone_id(req.drone_id().numeric_id(), RB)) return false;

    switch(apiMap[req.msgType()]) {
        case 0: {   /* VELOCITY */
        // RB SIDE
            // auto RetVel = RB->getCurrVel();
            // auto RetPos = RB->getCurrPos();
            // // can get a ros::Time back from these messages now
            // Res.vec3().x = RetVel.velocity.x;
            // Res.vec3().y = RetVel.velocity.y;
            // Res.vec3().z = RetVel.velocity.z;
            // Res.yaw_rate() = RetVel.yawRate;
            // Res.forward_x() = cos(-RetPos.yaw);
            // Res.forward_y() = sin(-RetPos.yaw);
            ROS_INFO_STREAM("Server completed get data service of type: " << req.msgType());
            return true;
        } break;
        case 1: {   /* POSITION */
        // RB SIDE
            // auto RetVel = RB->getCurrVel();
            // auto RetPos = RB->getCurrPos();
            // // can get a ros::Time back from these messages now
            // Res.vec3().x = RetPos.position.x;
            // Res.vec3().y = RetPos.position.y;
            // Res.vec3().z = RetPos.position.z;
            // Res.yaw_rate() = RetVel.yawRate;
            // Res.forward_x() = cos(-RetPos.yaw);
            // Res.forward_y() = sin(-RetPos.yaw);
            ROS_INFO_STREAM("Server completed get data service of type: " << req.msgType());
            return true;
        } break;
        case 7: {   /* GET_HOME */
        // RB SIDE
            geometry_msgs::Vector3 Pos = RB->get_home_coordinates();
            res.vec3().x = Pos.x;
            res.vec3().y = Pos.y;
            res.vec3().z = Pos.z;
            ROS_INFO_STREAM("Server completed get data service of type: " << req.msgType());
            return true;
        } break;
        case 9: {   /* ORIENTATION */
        // RB SIDE

            ROS_INFO_STREAM("Server completed get data service of type: " << req.msgType());
            return true;
        } break;
        case 10: {  /* TIME */
        // SERVER SIDE
            res.vec3().x = desiredLoopRate;
            res.vec3().y = achievedLoopRate;
            res.vec3().z = motionCaptureUpdateRate;
            res.forward_x() = timeToUpdateDrones;
            res.forward_y() = waitTime;
            ROS_INFO_STREAM("Server completed get data service of type: " << req.msgType());
            return true;
        } break;
        default: {
            ROS_ERROR_STREAM("The API command '" << req.msgType() << "' is not a valid command for dataFeedbackSRV");
        } break;
    }
    ROS_WARN_STREAM("Server failed get data service of type: " << req.msgType());
    return false;
}

bool drone_server::api_list_service(tf2_msgs::FrameGraph::Request &req, tf2_msgs::FrameGraph::Response &res) {
    /* encoding for the list service is done here without a helper class */
    /* check API functions documentation for clarity */
    res.frame_yaml = "";
    for (size_t i = 0; i < rigidbodyList.size(); i++) {
        if (rigidbodyList[i] == nullptr) continue;
        res.frame_yaml += std::to_string(i) + ":" + rigidbodyList[i]->get_name() + " ";
    }
    return true;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    drone_server server;
    server.run();

    return 0;
}