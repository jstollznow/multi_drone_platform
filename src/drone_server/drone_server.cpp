#include "drone_server.h"

#include <csignal>
#include <utility>

#include "multi_drone_platform/api_update.h"
#include "multi_drone_platform/add_drone.h"

#if POINT_SET_REG
#   define ICP_IMPL_INIT ,icpImplementation(&this->rigidbodyList, this->node)
#else
#   define ICP_IMPL_INIT
#endif /* POINT_SET_REG */

drone_server* globalDroneServer = nullptr;
bool globalShouldShutdown = false;
bool globalGoodShutDown = true;


drone_server::drone_server() : node(), loopRate(LOOP_RATE_HZ) ICP_IMPL_INIT
{
    node.setParam(SHUTDOWN_PARAM, false);
    inputAPISub = node.subscribe<geometry_msgs::TransformStamped> (SUB_TOPIC, 100, &drone_server::api_callback, this);
    emergencySub = node.subscribe<std_msgs::Empty> (EMERGENCY_TOPIC, 100, &drone_server::emergency_callback, this);
    std::string logTopic = NODE_NAME;
    logTopic += "/log";
    logPublisher = node.advertise<multi_drone_platform::log> (logTopic, 100);
    
    this->log(logger::INFO, "Initialising");
    
    dataServer = node.advertiseService(SRV_TOPIC, &drone_server::api_get_data_service, this);
    listServer = node.advertiseService(LIST_SRV_TOPIC, &drone_server::api_list_service, this);
    addDroneServer = node.advertiseService(ADD_DRONE_TOPIC, &drone_server::add_drone_service, this);
}

drone_server::~drone_server() {
    this->log(logger::INFO, "Shutting down drone server");
    this->shutdown();
}

void drone_server::shutdown() {
    for (size_t i = 0; i < rigidbodyList.size(); i++) {
        if (rigidbodyList[i] != nullptr) {
            rigidbodyList[i]->shutdown();
        }
    }
    
    /* sleep until drones have all landed */
    this->log(logger::INFO, "Waiting for drones to land...");
    bool allLanded = false;
    while (!allLanded) {
        allLanded = true;

        for (size_t i = 0; i < rigidbodyList.size(); i++) {
            if (rigidbodyList[i] != nullptr) {
                rigidbodyList[i]->update(rigidbodyList);
                if (rigidbodyList[i]->get_state() != rigidbody::flight_state::LANDED) {
                    allLanded = false;
                }
            }
        }

        if (!allLanded) {
            loopRate.sleep();
        }
    }

    this->log(logger::INFO, "Removing drones...");
    for (size_t i = 0; i < rigidbodyList.size(); i++) {
        remove_rigidbody(i);
    }
    rigidbodyList.clear();
}

bool drone_server::add_new_rigidbody(const std::string& pTag, std::vector<std::string> args) {
    rigidbody* RB;
    if (mdp_wrappers::create_new_rigidbody(pTag, rigidbodyList.size(), std::move(args), RB)) {
        /* update drone state on param server */

        /* indicate if the drone is a vflie or not, this is used for ICP */
        if (mdp_wrappers::get_drone_type_id(pTag) == droneTypeMap["vflie"]) {
            RB->isVflie = true;
        }

        rigidbodyList.push_back(RB);
        RB->mySpin.start();

        this->log(logger::DEBUG, "Successfully added '" + pTag + "'");
        return true;
    } else {
        this->log(logger::ERROR, "Unable to add '" + pTag + "', check if drone type naming is correct.");
        return false;
    }
}

void drone_server::remove_rigidbody(unsigned int pDroneID) {
    /* if pDroneID is a valid index and the object at that location is not null, delete */
    if (pDroneID < rigidbodyList.size()) {
        if (rigidbodyList[pDroneID] != nullptr) {
            
            this->log(logger::INFO, "Removing '" + rigidbodyList[pDroneID]->get_tag() + "'");

            rigidbodyList[pDroneID]->mySpin.stop();
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
    node.getParam(SHUTDOWN_PARAM, globalShouldShutdown);
    while (!globalShouldShutdown) {
        frameStart = ros::Time::now();
        /* do all the ros callback event stuff */
        ros::spinOnce();

        /* call update on every valid rigidbody */
        rigidbodyStart = ros::Time::now();
        for (auto & rigidbody : rigidbodyList) {
            if (rigidbody == nullptr) continue;

            rigidbody->update(rigidbodyList);
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
        if (!globalShouldShutdown) {
            if (!node.getParam(SHUTDOWN_PARAM, globalShouldShutdown)) {
                globalShouldShutdown = true;
            }
        }
    }
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
    mdp_translations::input_msg inputMsg((geometry_msgs::TransformStamped*)input.get());
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
    mdp_translations::drone_feedback_srv_req req(&pReq);
    mdp_translations::drone_feedback_srv_res res(&pRes);
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
        res.frame_yaml += std::to_string(i) + ":" + rigidbodyList[i]->get_tag() + " ";
    }
    return true;
}

bool drone_server::add_drone_service(multi_drone_platform::add_drone::Request &req, multi_drone_platform::add_drone::Response &res) {
    if (mdp_wrappers::get_drone_type_id(req.droneName) == 0) {
        res.success = false;
        res.reason = "Drone of type declared by tag '" + req.droneName + "' does not exist";
        return true;
    }
    for (auto& r : this->rigidbodyList) {
        if (r->get_tag() == req.droneName) {
            res.success = false;
            res.reason = "Drone with tag '" + req.droneName + "' already exists on the drone server";
            return true;
        }
    }

    this->add_new_rigidbody(req.droneName, req.arguments);
    res.success = true;
    return true;
}


void drone_server::log(logger::log_type logType, std::string message) {
    logger::post_log(logType, "Drone Server", logPublisher, std::move(message));
}


#define RESET   "\033[0m"
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */

void signal_handler(int sig) {
    globalGoodShutDown = false;
    globalShouldShutdown = true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME, ros::init_options::NoSigintHandler);

    signal(SIGINT, signal_handler);

    globalDroneServer = new drone_server;
    globalDroneServer->run();

    delete globalDroneServer;

    printf("Shutting down ROS\n");
    ros::shutdown();
    if (globalGoodShutDown) {
        printf(
                BOLDGREEN
                "Drone server shut down correctly, closing additional ros nodes.\n"
                "Ignore the following boxed red text:\n"
                RESET);
    } else {
        printf("\n\n\n"
               BOLDRED
               "WARNING: CTRL-C CALLED BY USER, DRONE SERVER MAY NOT SHUTDOWN CORRECTLY.\n"
               "TO PROPERLY SHUTDOWN SET THE ROS PARAM '/%s' TO true INSTEAD"
               RESET "\n\n\n", SHUTDOWN_PARAM);
    }
    return 0;
}