#include "../../include/user_api.h"

#include "ros/ros.h"
#include "boost/algorithm/string/split.hpp"

#include "../drone_server/drone_server_msg_translations.cpp"

namespace mdp_api {

struct node_data
{
    ros::NodeHandle* Node;
    ros::Publisher Pub;
    ros::ServiceClient DataClient;
    ros::ServiceClient ListClient;
} NodeData;



void initialise()
{
    int int_val = 0;
    ros::init(int_val, (char**)nullptr, FRAME_ID);

    NodeData.Node = new ros::NodeHandle("");

    NodeData.Pub = NodeData.Node->advertise<geometry_msgs::TransformStamped> ("mdp_api", 10);
    NodeData.DataClient = NodeData.Node->serviceClient<nav_msgs::GetPlan> ("mdp_api_data_srv");
    NodeData.ListClient = NodeData.Node->serviceClient<tf2_msgs::FrameGraph> ("mdp_api_list_srv");

    ROS_INFO("Initialised Client API Connection");
}

void terminate()
{
    delete NodeData.Node;
    ROS_INFO("Shutting Down Client API Connection");
}

std::vector<mdp_api::id> get_all_rigidbodies()
{
    tf2_msgs::FrameGraph Srv_data;
    NodeData.ListClient.call(Srv_data);

    std::vector<std::string> results;
    boost::split(results, Srv_data.response.frame_yaml, [](char c){return c == ' ';});

    std::vector<mdp_api::id> Vec;
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

    return Vec;
}

void set_drone_velocity(mdp_api::id pDroneID, float pVelX, float pVelY, float pVelZ, float pYawRate)
{
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numeric_id;
    Msg.msg_type() = "VELOCITY";

    Msg.posvel().x = pVelX;
    Msg.posvel().y = pVelY;
    Msg.posvel().z = pVelZ;
    Msg.yaw_rate() = pYawRate;

    NodeData.Pub.publish(Msg_data);
}

void set_drone_position(mdp_api::id pDroneID, float pPosX, float pPosY, float pPosZ, float pDuration, float pYaw)
{
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numeric_id;
    Msg.msg_type() = "POSITION";

    Msg.posvel().x = pPosX;
    Msg.posvel().y = pPosY;
    Msg.posvel().z = pPosZ;
    Msg.duration() = pDuration;
    Msg.yaw_rate() = pYaw;

    NodeData.Pub.publish(Msg_data);
}

position_data get_body_position(mdp_api::id pRigidbodyID)
{
    nav_msgs::GetPlan Srv_data;
    mdp::drone_feedback_srv Srv(&Srv_data);

    Srv.drone_id().numeric_id() = pRigidbodyID.numeric_id;
    Srv.msg_type() = "POSITION";

    position_data Data;
    NodeData.DataClient.call(Srv_data);
    Data.x = Srv.vec3().x;
    Data.y = Srv.vec3().y;
    Data.z = Srv.vec3().z;
    Data.yaw = Srv.yaw_rate();

    return Data;
}

velocity_data get_body_velocity(mdp_api::id pRigidbodyID)
{
    nav_msgs::GetPlan Srv_data;
    mdp::drone_feedback_srv Srv(&Srv_data);

    Srv.drone_id().numeric_id() = pRigidbodyID.numeric_id;
    Srv.msg_type() = "VELOCITY";
    NodeData.DataClient.call(Srv_data);

    velocity_data Data;
    Data.x = Srv.vec3().x;
    Data.y = Srv.vec3().y;
    Data.z = Srv.vec3().z;
    Data.yaw = Srv.yaw_rate();
    
    return Data;
}

void cmd_takeoff(mdp_api::id pDroneID)
{
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numeric_id;
    Msg.msg_type() = "TAKEOFF";

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


void set_home(mdp_api::id pDroneID, float pPosX, float pPosY, float pPosZ)
{
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numeric_id;
    Msg.msg_type() = "SET_HOME";

    Msg.posvel().x = pPosX;
    Msg.posvel().y = pPosY;
    Msg.posvel().z = pPosZ;

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
    NodeData.DataClient.call(Srv_data);

    position_data Data;
    Data.x = Srv.vec3().x;
    Data.y = Srv.vec3().y;
    Data.z = Srv.vec3().z;
    
    return Data;
}

void goto_home(mdp_api::id pDroneID)
{
    geometry_msgs::TransformStamped Msg_data;
    mdp::input_msg Msg(&Msg_data);

    Msg.drone_id().numeric_id() = pDroneID.numeric_id;
    Msg.msg_type() = "GOTO_HOME";

    NodeData.Pub.publish(Msg_data);
}


}