#include "user_api.h"

#include "ros/ros.h"
#include "boost/algorithm/string/split.hpp"
#include <unordered_map>

#include "../drone_server/drone_server_msg_translations.cpp"

#include "../objects/element_conversions.cpp"
#include "geometry_msgs/TwistStamped.h"

#define FRAME_ID "user_api"


namespace mdp_api {

struct drone_data {
    ros::Subscriber poseSubscriber;
    ros::Subscriber twistSubscriber;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::TwistStamped velocity;

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        this->pose = *msg;
    }

    void twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        this->velocity = *msg;
    }
};

struct node_data {
    ros::NodeHandle* Node;
    ros::Rate* LoopRate;
    int LoopRateValue;
    ros::Publisher Pub;
    ros::ServiceClient DataClient;
    ros::ServiceClient ListClient;
    std::unordered_map<uint32_t, drone_data> DroneData;
} NodeData;

void initialise(unsigned int pUpdateRate) {
    int int_val = 0;
    ros::init(int_val, (char**)nullptr, FRAME_ID);

    ROS_INFO("Initialising Client API Connection");

    NodeData.Node = new ros::NodeHandle();
    NodeData.LoopRate = new ros::Rate(pUpdateRate);
    NodeData.LoopRateValue = pUpdateRate;

    NodeData.Pub = NodeData.Node->advertise<geometry_msgs::TransformStamped> ("mdp_api", 2);
    NodeData.DataClient = NodeData.Node->serviceClient<nav_msgs::GetPlan> ("mdp_api_data_srv");
    NodeData.ListClient = NodeData.Node->serviceClient<tf2_msgs::FrameGraph> ("mdp_api_list_srv");

    sleep(1);

    ROS_INFO("Initialised Client API Connection");
}

void terminate() {
    ROS_INFO("Shutting Down Client API Connection");
    // land all active drones
    auto drones = get_all_rigidbodies();
    for (size_t i = 0; i < drones.size(); i++) {
        if (get_state({i, ""}) != "LANDED")
            cmd_land(drones[i]);
    }
    for (size_t i = 0; i < drones.size(); i++) {
        sleep_until_idle(drones[i]);
    }

    ROS_INFO("Finished Client API Connection");
    delete NodeData.LoopRate;
    delete NodeData.Node;
    ros::shutdown();
}

std::vector<mdp_api::id> get_all_rigidbodies() {
    tf2_msgs::FrameGraph Srv_data;

    std::vector<mdp_api::id> Vec;
    if (NodeData.ListClient.call(Srv_data)) {
        std::vector<std::string> results;
        boost::split(results, Srv_data.response.frame_yaml, [](char c){return c == ' ';});

        for (std::string& str : results) {
            if (str.length() > 0) {
                std::vector<std::string> id_str;
                boost::split(id_str, str, [](char c){return c == ':';});
                mdp_api::id ID;
                ID.numericID = atoi(id_str[0].c_str());
                ID.name = id_str[1];
                Vec.push_back(ID);
            }
        }

        // add drone data for each drone
        for (size_t i = 0; i < Vec.size(); i++) {
            if (NodeData.DroneData.count(Vec[i].numericID) == 0) {
                /* create drone_data struct and init ros Subscribers */
                NodeData.DroneData[Vec[i].numericID] = {};

                NodeData.DroneData[Vec[i].numericID].poseSubscriber = NodeData.Node->subscribe<geometry_msgs::PoseStamped>(
                    "mdp/drone_" + std::to_string(Vec[i].numericID) + "/pose", 
                    1, 
                    &drone_data::pose_callback, 
                    &NodeData.DroneData[Vec[i].numericID]);
                NodeData.DroneData[Vec[i].numericID].twistSubscriber = NodeData.Node->subscribe<geometry_msgs::TwistStamped>(
                    "mdp/drone_" + std::to_string(Vec[i].numericID) + "/velocity", 
                    1, 
                    &drone_data::twist_callback, 
                    &NodeData.DroneData[Vec[i].numericID]);
            }
        }
    } else {
        ROS_WARN("Failed to call api list service");
    }

    return Vec;
}

double encode_relative_array_to_double(bool relative, bool keepHeight) {
    return ((1.0 * relative) + (2.0 * keepHeight));
}

void set_drone_velocity(mdp_api::id pDroneID, mdp_api::velocity_msg pMsg) {
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numericID;
    Msg.msg_type() = "VELOCITY";

    Msg.pos_vel().x = pMsg.velocity[0];
    Msg.pos_vel().y = pMsg.velocity[1];
    Msg.pos_vel().z = pMsg.velocity[2];
    Msg.yaw_rate() = pMsg.yawRate;
    Msg.duration() = pMsg.duration;
    Msg.relative() = encode_relative_array_to_double(pMsg.relative, pMsg.keepHeight);

    NodeData.Pub.publish(Msg_data);
}

void set_drone_position(mdp_api::id pDroneID, mdp_api::position_msg pMsg) {
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numericID;
    Msg.msg_type() = "POSITION";

    Msg.pos_vel().x  = pMsg.position[0];
    Msg.pos_vel().y  = pMsg.position[1];
    Msg.pos_vel().z  = pMsg.position[2];
    Msg.duration()  = pMsg.duration;
    Msg.yaw()       = pMsg.yaw;
    Msg.relative()  = encode_relative_array_to_double(pMsg.relative, pMsg.keepHeight);

    NodeData.Pub.publish(Msg_data);
}

position_data get_position(mdp_api::id pRigidbodyID) {
    position_data Data;
    // if the drone id does not exist, return
    // @TODO: make this a value you can check for validity
    if (NodeData.DroneData.count(pRigidbodyID.numericID) == 0) return Data;

    auto Pose = &NodeData.DroneData[pRigidbodyID.numericID].pose;
    Data.timeStampNsec =    Pose->header.stamp.toNSec();
    Data.x =                Pose->pose.position.x;
    Data.y =                Pose->pose.position.y;
    Data.z =                Pose->pose.position.z;
    Data.yaw =              mdp_conversions::get_yaw_from_pose(Pose->pose);
    return Data;
}

velocity_data get_velocity(mdp_api::id pRigidbodyID) {
    velocity_data Data;
    // if the drone id does not exist, return
    if (NodeData.DroneData.count(pRigidbodyID.numericID) == 0) return Data;

    auto Vel = &NodeData.DroneData[pRigidbodyID.numericID].velocity;
    Data.timeStampNsec =    Vel->header.stamp.toNSec();
    Data.x =                Vel->twist.linear.x;
    Data.y =                Vel->twist.linear.y;
    Data.z =                Vel->twist.linear.z;
    Data.yaw =              Vel->twist.angular.y;
    return Data;
}

void cmd_takeoff(mdp_api::id pDroneID, float pHeight, float pDuration) {
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numericID;
    Msg.msg_type() = "TAKEOFF";
    Msg.pos_vel().z = pHeight;
    Msg.duration() = pDuration;

    NodeData.Pub.publish(Msg_data);
}

void cmd_land(mdp_api::id pDroneID) {
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numericID;
    Msg.msg_type() = "LAND";

    NodeData.Pub.publish(Msg_data);
}

void cmd_emergency(mdp_api::id pDroneID) {
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numericID;
    Msg.msg_type() = "EMERGENCY";

    NodeData.Pub.publish(Msg_data);
}

void cmd_hover(mdp_api::id pDroneID) {
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numericID;
    Msg.msg_type() = "HOVER";
    Msg.duration() = 10.0f;

    NodeData.Pub.publish(Msg_data);
}


void set_home(mdp_api::id pDroneID, mdp_api::position_msg pMsg) {
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numericID;
    Msg.msg_type() = "SET_HOME";

    Msg.pos_vel().x = pMsg.position[0];
    Msg.pos_vel().y = pMsg.position[1];
    Msg.pos_vel().z = pMsg.position[2];

    Msg.relative() = encode_relative_array_to_double(pMsg.relative, pMsg.keepHeight);
    Msg.yaw() = pMsg.yaw;

    NodeData.Pub.publish(Msg_data);
}

position_data get_home(mdp_api::id pDroneID) {
    nav_msgs::GetPlan Srv_data;
    mdp::drone_feedback_srv Srv(&Srv_data);

    mdp::id ID;
    ID.numeric_id() = pDroneID.numericID;

    Srv.drone_id().numeric_id() = pDroneID.numericID;
    Srv.msg_type() = "GET_HOME";

    position_data Data;
    if (NodeData.DataClient.call(Srv_data)) {
        Data.x = Srv.vec3().x;
        Data.y = Srv.vec3().y;
        Data.z = Srv.vec3().z;
    } else {
        ROS_WARN("Failed to call api data service");
    }
    
    return Data;
}

void go_to_home(mdp_api::id pDroneID, float duration, float pHeight) {
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numericID;
    Msg.msg_type() = "GOTO_HOME";
    Msg.pos_vel().z = pHeight;
    Msg.duration() = duration;

    Msg.relative() = encode_relative_array_to_double(false, (pHeight < 0.0f));

    NodeData.Pub.publish(Msg_data);
}

void set_drone_server_update_frequency(float pUpdateFrequency) {
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.msg_type() = "DRONE_SERVER_FREQ";
    Msg.pos_vel().x = pUpdateFrequency;

    NodeData.Pub.publish(Msg_data);
}

timings get_operating_frequencies() {
    nav_msgs::GetPlan Srv_data;
    mdp::drone_feedback_srv Srv(&Srv_data);

    Srv.msg_type() = "TIME";

    timings Data;
    if (NodeData.DataClient.call(Srv_data)) {
        Data.desDroneServerUpdateRate = Srv.vec3().x;
        Data.actualDroneServerUpdateRate = Srv.vec3().y;
        Data.moCapUpdateRate = Srv.vec3().z;
        Data.timeToUpdateDrones = Srv.forward_x();
        Data.waitTimePerFrame = Srv.forward_y();
    } else {
        ROS_WARN("Failed to call api data service");
    }
    
    return Data;
}

void spin_once() {
    NodeData.LoopRate->sleep();
    if (ros::ok())
        ros::spinOnce();
}

int rate() {
    return NodeData.LoopRateValue;
}

void sleep_until_idle(mdp_api::id pDroneID) {
    ROS_INFO("Sleeping until drone '%s' goes idle", pDroneID.name.c_str());
    
    /* wait 1 frame (so that states can update on the server side) */
    NodeData.LoopRate->reset();
    NodeData.LoopRate->sleep();

    std::string state_param = "mdp/drone_" + std::to_string(pDroneID.numericID) + "/state";
    std::string drone_state = "";
    if (!ros::param::get(state_param, drone_state)) {
        ROS_WARN("Failed to get current state of drone id: %d", pDroneID.numericID);
        return;
    }
    while (true) {
        if (drone_state == "DELETED") break;
        if (drone_state == "LANDED") break;
        if (drone_state == "IDLE") break;

        spin_once();
        ros::param::get(state_param, drone_state);
    }
}

std::string get_state(mdp_api::id pDroneID) {
    std::string state_param = "mdp/drone_" + std::to_string(pDroneID.numericID) + "/state";
    std::string drone_state;
    if (ros::param::get(state_param, drone_state)) {
        return drone_state;
    } else {
        ROS_WARN("Failed to get current state of drone id: %d", pDroneID.numericID);
        return "DELETED";
    }
}

}