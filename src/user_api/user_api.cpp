#include "../../include/user_api.h"

#include "ros/ros.h"
#include "boost/algorithm/string/split.hpp"

#include "../drone_server/drone_server_msg_translations.cpp"

#define FRAME_ID "user_api"

namespace mdp_api {

struct node_data
{
    ros::NodeHandle* Node;
    ros::Rate* LoopRate;
    int LoopRateValue;
    ros::Publisher Pub;
    ros::ServiceClient DataClient;
    ros::ServiceClient ListClient;
} NodeData;



void position_msg::set_as_relative(bool pValue)
{
    this->relative = {pValue,pValue,pValue};
}
void position_msg::set_target(mdp_api::id pTarget)
{
    this->target_id = pTarget.numeric_id;
}
void position_msg::rem_target()
{
    this->target_id = -1;
}
int position_msg::get_target_id() const
{
    return this->target_id;
}
void velocity_msg::set_as_relative(bool pValue)
{
    this->relative = {pValue,pValue,pValue};
}

void initialise(unsigned int pUpdateRate)
{
    int int_val = 0;
    ros::init(int_val, (char**)nullptr, FRAME_ID);

    ROS_INFO("Initialising Client API Connection");

    NodeData.Node = new ros::NodeHandle();

    NodeData.LoopRate = new ros::Rate(pUpdateRate);
    NodeData.LoopRateValue = pUpdateRate;

    NodeData.Pub = NodeData.Node->advertise<geometry_msgs::TransformStamped> ("mdp_api", 10);
    NodeData.DataClient = NodeData.Node->serviceClient<nav_msgs::GetPlan> ("mdp_api_data_srv");
    NodeData.ListClient = NodeData.Node->serviceClient<tf2_msgs::FrameGraph> ("mdp_api_list_srv");

    sleep(1);

    ROS_INFO("Initialised Client API Connection");
}

void terminate()
{
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
}

std::vector<mdp_api::id> get_all_rigidbodies()
{
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
                ID.numeric_id = atoi(id_str[0].c_str());
                ID.name = id_str[1];
                Vec.push_back(ID);
            }
        }
    } else {
        ROS_WARN("Failed to call api list service");
    }

    return Vec;
}

double encode_relative_array_to_double(std::array<bool, 3> pArray)
{
    return ((1.0 * pArray[0]) + (2.0 * pArray[1]) + (4.0 * pArray[2]));
}

void set_drone_velocity(mdp_api::id pDroneID, mdp_api::velocity_msg pMsg)
{
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numeric_id;
    Msg.msg_type() = "VELOCITY";

    Msg.posvel().x = pMsg.velocity[0];
    Msg.posvel().y = pMsg.velocity[1];
    Msg.posvel().z = pMsg.velocity[2];
    Msg.yaw_rate() = pMsg.yaw_rate;
    Msg.duration() = pMsg.duration;
    Msg.relative() = encode_relative_array_to_double(pMsg.relative);

    NodeData.Pub.publish(Msg_data);
}

void set_drone_position(mdp_api::id pDroneID, mdp_api::position_msg pMsg)
{
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numeric_id;
    Msg.msg_type() = "POSITION";

    Msg.posvel().x  = pMsg.position[0];
    Msg.posvel().y  = pMsg.position[1];
    Msg.posvel().z  = pMsg.position[2];
    Msg.duration()  = pMsg.duration;
    Msg.yaw()       = pMsg.yaw;
    Msg.relative()  = encode_relative_array_to_double(pMsg.relative);
    Msg.target()    = (double)pMsg.get_target_id();

    NodeData.Pub.publish(Msg_data);
}

position_data get_body_position(mdp_api::id pRigidbodyID)
{
    nav_msgs::GetPlan Srv_data;
    mdp::drone_feedback_srv Srv(&Srv_data);

    Srv.drone_id().numeric_id() = pRigidbodyID.numeric_id;
    Srv.msg_type() = "POSITION";

    position_data Data;
    if (NodeData.DataClient.call(Srv_data)) {
        Data.x = Srv.vec3().x;
        Data.y = Srv.vec3().y;
        Data.z = Srv.vec3().z;
        Data.yaw = Srv.yaw_rate();
    } else {
        ROS_WARN("Failed to call api data service");
    }

    return Data;
}

velocity_data get_body_velocity(mdp_api::id pRigidbodyID)
{
    nav_msgs::GetPlan Srv_data;
    mdp::drone_feedback_srv Srv(&Srv_data);

    Srv.drone_id().numeric_id() = pRigidbodyID.numeric_id;
    Srv.msg_type() = "VELOCITY";
    velocity_data Data;
    if (NodeData.DataClient.call(Srv_data)) {
        Data.x = Srv.vec3().x;
        Data.y = Srv.vec3().y;
        Data.z = Srv.vec3().z;
        Data.yaw = Srv.yaw_rate();
    } else {
        ROS_WARN("Failed to call api data service");
    }
    
    return Data;
}

void cmd_takeoff(mdp_api::id pDroneID, float pHeight, float pDuration)
{
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numeric_id;
    Msg.msg_type() = "TAKEOFF";
    Msg.posvel().z = pHeight;
    Msg.duration() = pDuration;

    NodeData.Pub.publish(Msg_data);
}

void cmd_land(mdp_api::id pDroneID)
{
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numeric_id;
    Msg.msg_type() = "LAND";

    NodeData.Pub.publish(Msg_data);
}

void cmd_emergency(mdp_api::id pDroneID)
{
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numeric_id;
    Msg.msg_type() = "EMERGENCY";

    NodeData.Pub.publish(Msg_data);
}

void cmd_hover(mdp_api::id pDroneID)
{
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numeric_id;
    Msg.msg_type() = "HOVER";

    NodeData.Pub.publish(Msg_data);
}


void set_home(mdp_api::id pDroneID, mdp_api::position_msg pMsg)
{
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numeric_id;
    Msg.msg_type() = "SET_HOME";

    Msg.posvel().x = pMsg.position[0];
    Msg.posvel().y = pMsg.position[1];
    Msg.posvel().z = pMsg.position[2];

    Msg.relative() = encode_relative_array_to_double(pMsg.relative);
    Msg.yaw() = pMsg.yaw;

    NodeData.Pub.publish(Msg_data);
}

position_data get_home(mdp_api::id pDroneID)
{
    nav_msgs::GetPlan Srv_data;
    mdp::drone_feedback_srv Srv(&Srv_data);

    mdp::id ID;
    ID.numeric_id() = pDroneID.numeric_id;

    Srv.drone_id().numeric_id() = pDroneID.numeric_id;
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

void goto_home(mdp_api::id pDroneID, float pHeight)
{
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numeric_id;
    Msg.msg_type() = "GOTO_HOME";
    Msg.posvel().z = pHeight;

    NodeData.Pub.publish(Msg_data);
}

void set_drone_server_update_frequency(float pUpdateFrequency)
{
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.msg_type() = "DRONE_SERVER_FREQ";
    Msg.posvel().x = pUpdateFrequency;

    NodeData.Pub.publish(Msg_data);
}

timings get_operating_frequencies()
{
    nav_msgs::GetPlan Srv_data;
    mdp::drone_feedback_srv Srv(&Srv_data);

    Srv.msg_type() = "TIME";

    timings Data;
    if (NodeData.DataClient.call(Srv_data)) {
        Data.desired_drone_server_update_rate = Srv.vec3().x;
        Data.achieved_drone_server_update_rate = Srv.vec3().y;
        Data.motion_capture_update_rate = Srv.vec3().z;
        Data.time_to_update_drones = Srv.forward_x();
        Data.wait_time_per_frame = Srv.forward_y();
    } else {
        ROS_WARN("Failed to call api data service");
    }
    
    return Data;
}

void spin_once()
{
    NodeData.LoopRate->sleep();
}

int rate()
{
    return NodeData.LoopRateValue;
}

void sleep_until_idle(mdp_api::id pDroneID)
{
    ROS_INFO("Sleeping until drone '%s' goes idle", pDroneID.name.c_str());
    NodeData.LoopRate->sleep();
    NodeData.LoopRate->sleep();
    std::string state_param = "mdp/drone_" + std::to_string(pDroneID.numeric_id) + "/state";
    std::string drone_state = "";
    if (!ros::param::get(state_param, drone_state)) {
        ROS_WARN("Failed to get current state of drone id: %d", pDroneID.numeric_id);
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

std::string get_state(mdp_api::id pDroneID)
{
    std::string state_param = "mdp/drone_" + std::to_string(pDroneID.numeric_id) + "/state";
    std::string drone_state;
    if (ros::param::get(state_param, drone_state)) {
        return drone_state;
    } else {
        ROS_WARN("Failed to get current state of drone id: %d", pDroneID.numeric_id);
        return "DELETED";
    }
}


}