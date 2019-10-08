#include "rigidBody.h"
#include "ros/ros.h"
#include "elementConversions.cpp"

void rigidBody::initialise()
{

}



rigidBody::rigidBody(std::string tag, bool controllable = false)
{
    this->platform_id = global_id++;
    this->moCapTag = tag;
    this->controllable = controllable;
    float init[3] = {0.0, 0.0, 0.0};
    std::string optiTop = "/optitrack/" + tag;
    
    moCapNode.Node = new ros::NodeHandle();
    moCapNode.Sub = moCapNode.Node->subscribe<geometry_msgs::PoseStamped>(optiTop, 10,&rigidBody::addMotionCapture);
    ROS_INFO("Subscribing to %s for motion capture", optiTop);
    
}

rigidBody::~rigidBody()
{
    delete moCapNode.Node;
    ROS_INFO("Shutting down rigid body %d:%s",platform_id,moCapTag);
}

bool rigidBody::getControllable()
{
    return this->controllable;
}

geometry_msgs::Vector3 rigidBody::vec3PosConvert(geometry_msgs::Pose& pos)
{
    geometry_msgs::Vector3 returnPos;
    returnPos.x = pos.position.x;
    returnPos.y = pos.position.y;
    returnPos.z = pos.position.z;
    return returnPos;
}
float rigidBody::getYaw(geometry_msgs::Pose& pos)
{
    return mdp_conversions::toEuler(pos.orientation).Yaw;
}
returnPos rigidBody::getCurrPos()
{
    // returns 0 duration
    float duration = 0;
    return {vec3PosConvert(currPos), getYaw(currPos), duration};
}

returnVel rigidBody::getCurrVel()
{
    float duration = 0;
    return {currVel.linear, currVel.angular.z, duration};
}

returnPos rigidBody::getDesPos()
{
    return {vec3PosConvert(desPos), getYaw(desPos), commandDuration};
}

void rigidBody::setDesPos(geometry_msgs::Vector3 pos, float yaw, float duration)
{
    return;
}

returnVel rigidBody::getDesVel()
{
    return {desVel.linear, desVel.angular.z, commandDuration};
}

void rigidBody::setDesVel(geometry_msgs::Vector3 vel, float yawRate, float duration)
{
    desVel.linear = vel;
    desVel.angular.z = yawRate;
    commandDuration = duration;
}

geometry_msgs::Vector3 rigidBody::getHomePos()
{
    return homePos;
}   

void rigidBody::setHomePos(geometry_msgs::Vector3 pos)
{
    homePos = pos;
}

void rigidBody::calcVel()
{
    ROS_INFO("Calculating Velocity");

    geometry_msgs::PoseStamped lastPos = motionCapture.front();
    motionCapture.erase(motionCapture.begin());
    geometry_msgs::PoseStamped firstPos = motionCapture.front();
    currVel = mdp_conversions::calcVel(lastPos,firstPos);
}

void rigidBody::addMotionCapture(const geometry_msgs::PoseStamped& msg)
{
    motionCapture.push_back(msg);
    if (motionCapture.size() >= 2){ calcVel(); }
    
    currPos = motionCapture.front().pose;

}

geometry_msgs::PoseStamped rigidBody::getMotionCapture()
{
    return motionCapture.front();
}

void rigidBody::update(std::vector<rigidBody*>& rigidBodies)
{
    // control loop for stabilisation, velocity or position control check
}
