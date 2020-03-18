#include "user_api.h"

#include "ros/ros.h"
#include "boost/algorithm/string/split.hpp"
#include <unordered_map>

#include "../drone_server/drone_server_msg_translations.cpp"

#include "../drone_server/element_conversions.cpp"
#include "geometry_msgs/TwistStamped.h"

#define FRAME_ID "user_api"


namespace mdp {

/**
 * a data structure used internally to represent a drone object on the drone-server
 */
struct drone_data {
    ros::Subscriber poseSubscriber;
    ros::Subscriber twistSubscriber;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped velocity;

    /**
     * callback for pose related ros messages for this drone
     * @param msg
     */
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        this->pose = *msg;
    }

    /**
     * callback for velocity related ros messages for this drone
     * @param msg
     */
    void twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        this->velocity = *msg;
    }
};

/**
 * persistent memory structure used internally, constructed with call to initialise(), destructed
 * with call to terminate()
 */
struct node_data {
    ros::NodeHandle* node;
    ros::Rate* loopRate;
    ros::Publisher publisher;
    ros::ServiceClient dataClient;
    ros::ServiceClient listClient;
    std::unordered_map<uint32_t, drone_data> droneData;
}* nodeData;

void initialise(double pUpdateRate, std::string nodeName) {
    nodeData = new node_data;
    int intVal = 0;
    ros::init(intVal, (char**)nullptr, nodeName);

    ROS_INFO("Initialising Client API Connection");

    nodeData->node = new ros::NodeHandle();
    nodeData->loopRate = new ros::Rate(pUpdateRate);

    nodeData->publisher = nodeData->node->advertise<geometry_msgs::TransformStamped> ("mdp", 2);
    nodeData->dataClient = nodeData->node->serviceClient<nav_msgs::GetPlan> ("mdp_data_srv");
    nodeData->listClient = nodeData->node->serviceClient<tf2_msgs::FrameGraph> ("mdp_list_srv");

    sleep(1);

    ROS_INFO("Initialised Client API Connection");
}

void terminate() {
    ROS_INFO("Shutting Down Client API Connection");
    // land all active drones
    auto drones = get_all_rigidbodies();
    for (size_t i = 0; i < drones.size(); i++) {
        if (get_state({static_cast<uint32_t>(i), ""}) != "LANDED")
            cmd_land(drones[i]);
    }
    for (const auto & drone : drones) {
        sleep_until_idle(drone);
    }

    ROS_INFO("Finished Client API Connection");
    delete nodeData->loopRate;
    delete nodeData->node;
    delete nodeData;
    ros::shutdown();
}

std::vector<mdp::id> get_all_rigidbodies() {
    tf2_msgs::FrameGraph srvData;

    std::vector<mdp::id> vec;
    if (nodeData->listClient.call(srvData)) {
        std::vector<std::string> results;
        boost::split(results, srvData.response.frame_yaml, [](char c){return c == ' ';});

        for (std::string& str : results) {
            if (str.length() > 0) {
                std::vector<std::string> idStr;
                boost::split(idStr, str, [](char c){return c == ':';});
                mdp::id newId;
                newId.numericID = atoi(idStr[0].c_str());
                newId.name = idStr[1];
                vec.push_back(newId);
            }
        }

        // add drone data for each drone
        for (size_t i = 0; i < vec.size(); i++) {
            if (nodeData->droneData.count(vec[i].numericID) == 0) {
                /* create drone_data struct and init ros Subscribers */
                nodeData->droneData[vec[i].numericID] = {};

                nodeData->droneData[vec[i].numericID].poseSubscriber = nodeData->node->subscribe<geometry_msgs::PoseStamped>(
                    "mdp/drone_" + std::to_string(vec[i].numericID) + "/curr_pose",
                    1, 
                    &drone_data::pose_callback, 
                    &nodeData->droneData[vec[i].numericID]);
                nodeData->droneData[vec[i].numericID].twistSubscriber = nodeData->node->subscribe<geometry_msgs::TwistStamped>(
                    "mdp/drone_" + std::to_string(vec[i].numericID) + "/curr_twist",
                    1, 
                    &drone_data::twist_callback, 
                    &nodeData->droneData[vec[i].numericID]);
            }
        }
    } else {
        ROS_WARN("Failed to call api list service");
    }

    return vec;
}

double encode_relative_array_to_double(bool relative, bool keepHeight) {
    return ((1.0 * relative) + (2.0 * keepHeight));
}

void set_drone_velocity(const mdp::id& pDroneID, mdp::velocity_msg pMsg) {
    geometry_msgs::TransformStamped msgData;
    mdp_translations::input_msg inputMsg(&msgData);

    inputMsg.drone_id().numeric_id() = pDroneID.numericID;
    inputMsg.msg_type() = "VELOCITY";

    inputMsg.pos_vel().x = pMsg.velocity[0];
    inputMsg.pos_vel().y = pMsg.velocity[1];
    inputMsg.pos_vel().z = pMsg.velocity[2];
    inputMsg.yaw_rate() = pMsg.yawRate;
    inputMsg.duration() = pMsg.duration;
    inputMsg.relative() = encode_relative_array_to_double(pMsg.relative, pMsg.keepHeight);

    nodeData->publisher.publish(msgData);
}

void set_drone_position(const mdp::id& pDroneID, mdp::position_msg pMsg) {
    geometry_msgs::TransformStamped msgData;
    mdp_translations::input_msg inputMsg(&msgData);

    inputMsg.drone_id().numeric_id() = pDroneID.numericID;
    inputMsg.msg_type() = "POSITION";

    inputMsg.pos_vel().x  = pMsg.position[0];
    inputMsg.pos_vel().y  = pMsg.position[1];
    inputMsg.pos_vel().z  = pMsg.position[2];
    inputMsg.duration()  = pMsg.duration;
    inputMsg.yaw()       = pMsg.yaw;
    inputMsg.relative()  = encode_relative_array_to_double(pMsg.relative, pMsg.keepHeight);

    nodeData->publisher.publish(msgData);
}

position_data get_position(const mdp::id& pRigidbodyID) {
    position_data data;
    // if the drone id does not exist, return
    // @TODO: make this a value you can check for validity
    if (nodeData->droneData.count(pRigidbodyID.numericID) == 0) return data;

    auto Pose = &nodeData->droneData[pRigidbodyID.numericID].pose;
    data.respectiveID =     pRigidbodyID;
    data.timeStampNsec =    Pose->header.stamp.toNSec();
    data.x =                Pose->pose.position.x;
    data.y =                Pose->pose.position.y;
    data.z =                Pose->pose.position.z;
    data.yaw =              mdp_conversions::get_yaw_from_pose(Pose->pose);
    return data;
}

velocity_data get_velocity(const mdp::id& pRigidbodyID) {
    velocity_data data;
    // if the drone id does not exist, return
    if (nodeData->droneData.count(pRigidbodyID.numericID) == 0) return data;

    auto Vel = &nodeData->droneData[pRigidbodyID.numericID].velocity;
    data.respectiveID =     pRigidbodyID;
    data.timeStampNsec =    Vel->header.stamp.toNSec();
    data.x =                Vel->twist.linear.x;
    data.y =                Vel->twist.linear.y;
    data.z =                Vel->twist.linear.z;
    data.yawRate =          Vel->twist.angular.y;
    return data;
}

void cmd_takeoff(const mdp::id& pDroneID, float pHeight, float pDuration) {
    geometry_msgs::TransformStamped msgData;
    mdp_translations::input_msg inputMsg(&msgData);

    inputMsg.drone_id().numeric_id() = pDroneID.numericID;
    inputMsg.msg_type() = "TAKEOFF";
    inputMsg.pos_vel().z = pHeight;
    inputMsg.duration() = pDuration;

    nodeData->publisher.publish(msgData);
}

void cmd_land(const mdp::id& pDroneID, float duration) {
    geometry_msgs::TransformStamped msgData;
    mdp_translations::input_msg inputMsg(&msgData);

    inputMsg.drone_id().numeric_id() = pDroneID.numericID;
    inputMsg.msg_type() = "LAND";
    inputMsg.duration() = duration;

    nodeData->publisher.publish(msgData);
}

void cmd_emergency(const mdp::id& pDroneID) {
    geometry_msgs::TransformStamped msgData;
    mdp_translations::input_msg inputMsg(&msgData);

    inputMsg.drone_id().numeric_id() = pDroneID.numericID;
    inputMsg.msg_type() = "EMERGENCY";

    nodeData->publisher.publish(msgData);
}

void cmd_hover(const mdp::id& pDroneID, float duration) {
    geometry_msgs::TransformStamped msgData;
    mdp_translations::input_msg inputMsg(&msgData);

    inputMsg.drone_id().numeric_id() = pDroneID.numericID;
    inputMsg.msg_type() = "HOVER";
    inputMsg.duration() = duration;

    nodeData->publisher.publish(msgData);
}


void set_home(const mdp::id& pDroneID, mdp::position_msg pMsg) {
    geometry_msgs::TransformStamped msgData;
    mdp_translations::input_msg inputMsg(&msgData);

    inputMsg.drone_id().numeric_id() = pDroneID.numericID;
    inputMsg.msg_type() = "SET_HOME";

    inputMsg.pos_vel().x = pMsg.position[0];
    inputMsg.pos_vel().y = pMsg.position[1];
    inputMsg.pos_vel().z = pMsg.position[2];

    inputMsg.relative() = encode_relative_array_to_double(pMsg.relative, pMsg.keepHeight);
    inputMsg.yaw() = pMsg.yaw;

    nodeData->publisher.publish(msgData);
}

position_data get_home(const mdp::id& pDroneID) {
    nav_msgs::GetPlan srvData;
    mdp_translations::drone_feedback_srv feedbackSrv(&srvData);

    mdp_translations::id newId;
    newId.numeric_id() = pDroneID.numericID;

    feedbackSrv.drone_id().numeric_id() = pDroneID.numericID;
    feedbackSrv.msg_type() = "GET_HOME";

    position_data posData;
    posData.respectiveID = pDroneID;
    if (nodeData->dataClient.call(srvData)) {
        posData.timeStampNsec = ros::Time::now().toNSec();
        posData.x = feedbackSrv.vec3().x;
        posData.y = feedbackSrv.vec3().y;
        posData.z = feedbackSrv.vec3().z;
    } else {
        ROS_WARN("Failed to call api data service");
    }
    
    return posData;
}

void go_to_home(const mdp::id& pDroneID, float duration, float pHeight) {
    geometry_msgs::TransformStamped msgData;
    mdp_translations::input_msg inputMsg(&msgData);

    inputMsg.drone_id().numeric_id() = pDroneID.numericID;
    inputMsg.msg_type() = "GOTO_HOME";
    inputMsg.pos_vel().z = pHeight;
    inputMsg.duration() = duration;

    inputMsg.relative() = encode_relative_array_to_double(false, (pHeight < 0.0f));

    nodeData->publisher.publish(msgData);
}

void set_drone_server_update_frequency(float pUpdateFrequency) {
    geometry_msgs::TransformStamped msgData;
    mdp_translations::input_msg inputMsg(&msgData);

    inputMsg.msg_type() = "DRONE_SERVER_FREQ";
    inputMsg.pos_vel().x = pUpdateFrequency;

    nodeData->publisher.publish(msgData);
}

timings get_operating_frequencies() {
    nav_msgs::GetPlan srvData;
    mdp_translations::drone_feedback_srv feedbackSrv(&srvData);

    feedbackSrv.msg_type() = "TIME";

    timings timingsData{};
    if (nodeData->dataClient.call(srvData)) {
        timingsData.timeStampNsec = ros::Time::now().toNSec();
        timingsData.desDroneServerUpdateRate = feedbackSrv.vec3().x;
        timingsData.actualDroneServerUpdateRate = feedbackSrv.vec3().y;
        timingsData.moCapUpdateRate = feedbackSrv.vec3().z;
        timingsData.timeToUpdateDrones = feedbackSrv.forward_x();
        timingsData.waitTimePerFrame = feedbackSrv.forward_y();
    } else {
        ROS_WARN("Failed to call api data service");
    }
    
    return timingsData;
}

void spin_once() {
    nodeData->loopRate->sleep();
    if (ros::ok())
        ros::spinOnce();
}

void sleep_until_idle(const mdp::id& pDroneID) {
    ROS_INFO("Sleeping until drone '%s' goes idle", pDroneID.name.c_str());
    
    /* wait 1 frame (so that states can update on the server side) */
    nodeData->loopRate->reset();
    nodeData->loopRate->sleep();

    std::string stateParam = "mdp/drone_" + std::to_string(pDroneID.numericID) + "/state";
    std::string droneState;
    if (!ros::param::get(stateParam, droneState)) {
        ROS_WARN("Failed to get current state of drone id: %d", pDroneID.numericID);
        return;
    }
    while (true) {
        if (droneState == "DELETED") break;
        if (droneState == "LANDED") break;
        if (droneState == "HOVERING") break;

        spin_once();
        if (!ros::param::get(stateParam, droneState)) {
            return;
        }
    }
}

std::string get_state(const mdp::id& pDroneID) {
    std::string stateParam = "mdp/drone_" + std::to_string(pDroneID.numericID) + "/state";
    std::string droneState;
    if (ros::param::get(stateParam, droneState)) {
        return droneState;
    } else {
        ROS_WARN("Failed to get current state of drone id: %d", pDroneID.numericID);
        return "DELETED";
    }
}

bool position_data::isValid() const {
    return (this->timeStampNsec > 0);
}

bool velocity_data::isValid() const {
    return (this->timeStampNsec > 0);
}

bool timings::isValid() const {
    return (this->timeStampNsec > 0);
}
}