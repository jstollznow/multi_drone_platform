#include "../../include/user_api.h"

#include "ros/ros.h"

#include "../drone_server/drone_server_msg_translations.cpp"

namespace mdp_api {

struct node_data
{
    ros::NodeHandle* Node;
    ros::Publisher Pub;
    ros::ServiceClient Client;
} NodeData;



void initialise(int argc, char **argv)
{
    ros::init(argc, argv, FRAME_ID);

    NodeData.Node = new ros::NodeHandle();

    NodeData.Pub = NodeData.Node->advertise<geometry_msgs::TransformStamped> (PUB_TOPIC, 10);
    NodeData.Client = NodeData.Node->serviceClient<nav_msgs::GetPlan> (FEEDBACK_TOPIC);

    ROS_INFO("Initialised Client API Connection");
}

void terminate()
{
    delete NodeData.Node;
    ROS_INFO("Shutting Down Client API Connection");
}

std::vector<mdp_api::id> get_all_rigidbodies()
{
    // @TODO: implement this functionality
    std::vector<mdp_api::id> Vec;
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

    mdp::id ID;
    ID.numeric_id() = pRigidbodyID.numeric_id;
    ID.name() = pRigidbodyID.name;

    Srv.setDroneID(ID.getData());
    Srv.msg_type() = "POSITION";
    NodeData.Client.call(Srv_data);

    position_data Data;
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

    mdp::id ID;
    ID.numeric_id() = pRigidbodyID.numeric_id;
    ID.name() = pRigidbodyID.name;

    Srv.setDroneID(ID.getData());
    Srv.msg_type() = "VELOCITY";
    NodeData.Client.call(Srv_data);

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
    ID.name() = pDroneID.name;

    Srv.setDroneID(ID.getData());
    Srv.msg_type() = "GET_HOME";
    NodeData.Client.call(Srv_data);

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