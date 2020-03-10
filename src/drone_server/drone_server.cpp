#include "drone_server.h"
#include "multi_drone_platform/api_update.h"


drone_server::drone_server() : node(), loopRate(LOOP_RATE_HZ) {
    inputAPISub = node.subscribe<geometry_msgs::TransformStamped> (SUB_TOPIC, 2, &drone_server::api_callback, this);
    emergencySub = node.subscribe<std_msgs::Empty> (EMERGENCY_TOPIC, 10, &drone_server::emergency_callback, this);
    std::string logTopic = NODE_NAME;
    logTopic += "/log";
    logPublisher = node.advertise<multi_drone_platform::log> (logTopic, 10);
    
    this->log(logger::INFO, "Initialising");
    
    dataServer = node.advertiseService(SRV_TOPIC, &drone_server::api_get_data_service, this);
    listServer = node.advertiseService(LIST_SRV_TOPIC, &drone_server::api_list_service, this);
    
    // std::string droneName1;
    // std::string droneName2;
    // if (Node.hasParam("cflie_test1"))
    // {
    //     Node.getParam("cflie_test1", droneName2);
    //     ROS_INFO("Adding %s", droneName2.c_str());
    //     addNewRigidbody(droneName2);
    // }
    int n = 4;
    for (int i = 0; i < n; i++) {
        std::string droneStr = "vflie_";
        if (i < 10) {
            droneStr += "0";
        }
        droneStr += std::to_string(i);
        add_new_rigidbody(droneStr);
    }
}

drone_server::~drone_server() {
    this->shutdown();
}

void drone_server::shutdown() {
    for (size_t i = 0; i < rigidbodyList.size(); i++) {
        remove_rigidbody(i);
    }
    rigidbodyList.clear();
    // is shutdown called before or after ROS is killed?
    // if after uncomment following line and comment printf
    // this->log(logger::INFO, "Shutting down");
    printf("Shutting down drone server\n");
}

// @TODO: implementation task on Trello
void drone_server::init_rigidbodies_from_VRPN() {
    this->log(logger::WARN, "Initialising drones from vrpn is \
    currently not supported, please add drones manually");
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

        this->log(logger::DEBUG, "Successfully added '" + pTag + "'");
    } else {
        this->log(logger::ERROR, "Unable to add '" + pTag + "', check if drone type naming is correct.");
    }
    return ID;
}

void drone_server::remove_rigidbody(unsigned int pDroneID) {
    /* if pDroneID is a valid index and the object at that location is not null, delete */
    if (pDroneID < rigidbodyList.size()) {
        if (rigidbodyList[pDroneID] != nullptr) {
            
            this->log(logger::WARN, "Removing '" + rigidbodyList[pDroneID]->get_name() + "'");

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
        this->log(logger::WARN, "Supplied ID is greater than size of rigidbody list: " + 
        std::to_string(pID) + " >= " + std::to_string(rigidbodyList.size()));
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

        // 5 Seconds
        if (timingPrint >= TIMING_UPDATE*LOOP_RATE_HZ) {
            float avgLoopRate = achievedLoopRate/timingPrint;
            float avgWaitTime = waitTime/timingPrint;
            float avgDroneUpdate = timeToUpdateDrones/timingPrint;
            timingPrint = 0;

            std::string loopInfo = "Avg. Loop Info--\n";
            loopInfo += "Actual [Hz]: " + std::to_string(avgLoopRate) + 
            ", Wait [s]: " + std::to_string(avgWaitTime) + ", Drones [s]: " 
            + std::to_string(avgDroneUpdate);

            this->log(logger::INFO, loopInfo);

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
    this->log(logger::ERROR, "EMERGENCY CALLED");
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
    this->log(logger::INFO, "Server recieved get data service of type: " + req.msgType());
    
    rigidbody* RB;
    if (!get_rigidbody_from_drone_id(req.drone_id().numeric_id(), RB)) return false;

    switch(apiMap[req.msgType()]) {
        case 7: {   /* GET_HOME */
        // RB SIDE
            geometry_msgs::Vector3 Pos = RB->get_home_coordinates();
            res.vec3().x = Pos.x;
            res.vec3().y = Pos.y;
            res.vec3().z = Pos.z;
            this->log(logger::DEBUG, "Server completed get data service of type: " + req.msgType());
            return true;
        } break;
        case 10: {  /* TIME */
        // SERVER SIDE
            res.vec3().x = desiredLoopRate;
            res.vec3().y = achievedLoopRate;
            res.vec3().z = motionCaptureUpdateRate;
            res.forward_x() = timeToUpdateDrones;
            res.forward_y() = waitTime;
            this->log(logger::DEBUG, "Server completed get data service of type: " + req.msgType());
            return true;
        } break;
        default: {
            this->log(logger::ERROR, "The API command '" + req.msgType() + "' is not a valid command for dataFeedbackSRV");
        } break;
    }
    this->log(logger::WARN, "Server failed get data service of type: " + req.msgType());
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

void drone_server::log(logger::log_type logType, std::string message) {
    logger::post_log(logType, "Drone Server", message, logPublisher);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    drone_server server;
    server.run();

    return 0;
}