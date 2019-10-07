

#include "multi_drone_platform/inputAPI.h"
#include "multi_drone_platform/movementFeedbackSRV.h"
#include "geometry_msgs/PoseStamped.h"

#include "../objects/nodeData.h"
#include "../objects/rigidBody.h"
#include<vector>

class droneServer
{
    public:
        std::vector<rigidBody> rigidBodies;
        node_data myNode;
        droneServer()
        {
            myNode.Node = new ros::NodeHandle();
        }
};