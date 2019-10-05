#include "rigidBody.h"
#include "ros/ros.h"
void rigidBody::initialise()
{
    
}

void rigidBody::calcVel()
{
    geometry_msgs::PoseStamped lastPos = motionCapture.front();
    motionCapture.erase(motionCapture.begin());
    geometry_msgs::PoseStamped firstPos = motionCapture.front();
    // velocity.linear.x = lastPos.pose.position.x - firstPos.pose.position.x;
    // velocity.linear.y = lastPos.pose.position.y - firstPos.pose.position.y;
    // velocity.linear.z = lastPos.pose.position.z - firstPos.pose.position.z;  
}

// constructor
rigidBody::rigidBody(std::string tag)
{
    this->platform_id = global_id++;
    this->optitrackTag = tag;
    float init[3] = {0.0, 0.0, 0.0};
    std::string subTopic = "/optitrack/" + tag; 
    myNode.Node = new ros::NodeHandle();
    myNode.Sub = myNode.Node->subscribe<geometry_msgs::PoseStamped>(subTopic, 10, &rigidBody::addMotionCapture, this);
}

// deconstructor
rigidBody::~rigidBody()
{
    delete myNode.Node;
}

void rigidBody::addMotionCapture(const geometry_msgs::PoseStamped msg)
{
    motionCapture.push_back(msg);
    if (motionCapture.size() >= 2)
    {
        calcVel();
    }
}

geometry_msgs::PoseStamped rigidBody::getMotionCapture()
{
    return motionCapture.front();
}