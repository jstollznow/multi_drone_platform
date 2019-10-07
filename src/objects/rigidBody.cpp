#include "rigidBody.h"
#include "ros/ros.h"
#include "velocity.cpp"


void rigidBody::initialise()
{

}

void rigidBody::calcVel()
{
    ROS_INFO("Calculating Velocity");

    geometry_msgs::PoseStamped lastPos = motionCapture.front();
    motionCapture.erase(motionCapture.begin());
    geometry_msgs::PoseStamped firstPos = motionCapture.front();
    currVel = mdp_velControl::calcVel(lastPos,firstPos);
}

// constructor
rigidBody::rigidBody(std::string tag)
{

    this->platform_id = global_id++;
    this->moCapTag = tag;
    float init[3] = {0.0, 0.0, 0.0};
    std::string optiTop = "/optitrack/" + tag; 
    std::string apiTop = "/api_input";
    
    moCapNode.Node = new ros::NodeHandle();
    moCapNode.Sub = moCapNode.Node->subscribe<geometry_msgs::PoseStamped> (optiTop, 10, &rigidBody::addMotionCapture, this);
    ROS_INFO("Subscribing to %s for motion capture", optiTop);

    apiNode.Node = new ros::NodeHandle();
    apiNode.Sub = apiNode.Node->subscribe<multi_drone_platform::inputAPI> (apiTop, 10, &rigidBody::APIUpdate, this);
    ROS_INFO("Subscribing to %s for API messages", apiTop);
    
    // optitrack update rate
    int loopFeq;
    ros::Rate LoopRate(loopFeq);
    ROS_INFO("Loop Freq is %d", loopFeq);

    while (ros::ok())
    {
        ros::spinOnce();
        LoopRate.sleep();
    }
}

// deconstructor
rigidBody::~rigidBody()
{
    delete moCapNode.Node;
    delete apiNode.Node;
    ROS_INFO("Shutting down rigid body %d:%s",platform_id,moCapTag);
}

multi_drone_platform::inputData rigidBody::getCurrPos()
{
    return currPos;
}

multi_drone_platform::inputData rigidBody::getCurrVel()
{
    return currVel;
}

multi_drone_platform::inputData rigidBody::getDesPos()
{
    return desPos;
}

void rigidBody::setDesPos(multi_drone_platform::inputData pos, float duration)
{
    desPos = pos;    
    commDur = duration;
}
        
multi_drone_platform::inputData rigidBody::getDesVel()
{
    return desVel;
}

void rigidBody::setDesVel(multi_drone_platform::inputData vel, float duration)
{
    desVel = vel;
    commDur = duration;
}

// for home pos, angular pos does not matter
geometry_msgs::Vector3 rigidBody::getHomePos()
{
    return homePos;
}

void rigidBody::setHomePos(geometry_msgs::Vector3 pos)
{
    homePos = pos;    
}

void rigidBody::addMotionCapture(const geometry_msgs::PoseStamped msg)
{
    motionCapture.push_back(msg);
    if (motionCapture.size() >= 2)
    {
        calcVel();
    }

    // update currPos
    geometry_msgs::PoseStamped lastPos = motionCapture.front();
    geometry_msgs::Vector3 angPos;
    angPos = mdp_velControl::getUpVector(lastPos.pose.orientation);
    
    currPos.posvel.x = lastPos.pose.position.x;
    currPos.posvel.y = lastPos.pose.position.y;
    currPos.posvel.z = lastPos.pose.position.z;

    currPos.forward.x = angPos.x;
    currPos.forward.y = angPos.y;
    currPos.forward.z = angPos.z;
}

geometry_msgs::PoseStamped rigidBody::getMotionCapture()
{
    return motionCapture.front();
}

void rigidBody::APIUpdate(const multi_drone_platform::inputAPI msg)
{
    char msgChar = msg.msg_type[0];
    multi_drone_platform::inputData desired = msg.data;
    float dur = msg.data.duration;
    switch (msgChar)
    {
    // Take-off
    case 'T':
        // take-off command
        break;

    // Eland
    case 'E':
        
        break;
    
    // Land
    case 'L':
        desired.posvel.z = 0;
        setDesPos(desired, dur);
        break;
    
    // Velocity
    case 'V':
        setDesVel(desired,dur);
        break;
    
    // Position
    case 'P':
        setDesPos(desired, dur);
        break;
    
    // Hover
    case 'H':
        // need to manage timing here, not sure how
        desired.posvel.x = 0.0;
        desired.posvel.y = 0.0;
        desired.posvel.z = 0.0;       
        setDesVel(desired, dur);
        break;
    
    // GoToHome
    case 'G':
        desired.posvel = getHomePos();
        // need to reflect a constant velocity
        // always travel to home at same velocity
        setDesPos(desired, dur);
        break;
    
    // Set Home
    case 'S':
        setHomePos(desired.posvel);
        break;    
    
    default:
        break;
    }
}