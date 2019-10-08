#include "../../include/user_api.h"

#include "ros/ros.h"

#include "multi_drone_platform/inputAPI.h"
#include "multi_drone_platform/movementFeedbackSRV.h"
#include "geometry_msgs/PoseStamped.h"

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

    NodeData.Pub = NodeData.Node->advertise<multi_drone_platform::inputAPI> (PUB_TOPIC, 1);
    NodeData.Client = NodeData.Node->serviceClient<multi_drone_platform::movementFeedbackSRV> (FEEDBACK_TOPIC);

    ROS_INFO("Initialised Client API Connection");
}

void terminate()
{
    delete NodeData.Node;
    ROS_INFO("Shutting Down Client API Connection");
}

std::vector<uint32_t> get_all_rigidbodies()
{
    // @TODO: implement this functionality
    std::vector<uint32_t> Vec;
    return Vec;
}

void set_drone_velocity(uint32_t pDroneID, float pVelX, float pVelY, float pVelZ, float pYawRate)
{
    multi_drone_platform::inputAPI Msg;

    Msg.drone_id.drone_id = pDroneID;
    Msg.msg_type = "VELOCITY";

    // Msg.movement.vec3.x = pVelX;
    // Msg.movement.vec3.y = pVelY;
    // Msg.movement.vec3.z = pVelZ;
    // Msg.movement.yaw = pYawRate;

    NodeData.Pub.publish(Msg);
}

void set_drone_position(uint32_t pDroneID, float pPosX, float pPosY, float pPosZ, float pDuration, float pYaw)
{
    // @FIX: we need to add Duration to the message
    multi_drone_platform::inputAPI Msg;

    Msg.drone_id.drone_id = pDroneID;
    Msg.msg_type = "POSITION";

    // Msg.movement.vec3.x = pPosX;
    // Msg.movement.vec3.y = pPosY;
    // Msg.movement.vec3.z = pPosZ;
    // Msg.movement.yaw = pYaw;

    NodeData.Pub.publish(Msg);
}

position_data get_body_position(uint32_t pRigidbodyID)
{
    multi_drone_platform::movementFeedbackSRV Srv;
    Srv.request.drone_id = pRigidbodyID;
    Srv.request.data_type = "POSITION";
    NodeData.Client.call(Srv);

    position_data Data;
    Data.droneID = pRigidbodyID;
    Data.x = Srv.response.vec3.x;
    Data.y = Srv.response.vec3.y;
    Data.z = Srv.response.vec3.z;
    Data.yaw = Srv.response.yaw;
    
    return Data;
}

velocity_data get_body_velocity(uint32_t pRigidbodyID)
{
    multi_drone_platform::movementFeedbackSRV Srv;
    Srv.request.drone_id = pRigidbodyID;
    Srv.request.data_type = "VELOCITY";
    NodeData.Client.call(Srv);

    position_data Data;
    Data.droneID = pRigidbodyID;
    Data.x = Srv.response.vec3.x;
    Data.y = Srv.response.vec3.y;
    Data.z = Srv.response.vec3.z;
    Data.yaw = Srv.response.yaw;
    
    return Data;
}

}