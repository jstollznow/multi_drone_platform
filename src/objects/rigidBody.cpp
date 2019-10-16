#include "rigidBody.h"
#include "ros/ros.h"
#include "elementConversions.cpp"


void rigidBody::initialise()
{

}
bool rigidBody::checkTopicValid(std::string topicName)
{
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (auto it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        if (info.name == topicName.c_str())
        {
            return true;
        }
    }
    return false;
}

rigidBody::rigidBody(std::string tag, bool controllable)
{
    this->tag = tag;
    
    // drone or obstacle
    this->controllable = controllable;

    // look for drone under tag namespace then vrpn output
    std::string optiTop = "/vrpn_client_node/" + tag + "/pose";

    // the same but in the crazyflie_server namespace
    std::string crazyflieTopic = "/" + tag + optiTop;

    desPos.position.x = 0.0f;
    desPos.position.y = 0.0f;
    desPos.position.z = 0.0f;
    
    commandDuration = 0.0f;
    droneHandle = ros::NodeHandle();
    motionSub = droneHandle.subscribe<geometry_msgs::PoseStamped>(optiTop, 10,&rigidBody::addMotionCapture, this);
    external_pose = droneHandle.advertise<geometry_msgs::PoseStamped>("/" + tag + "/external_pose", DEFAULT_QUEUE);
        
    ROS_INFO("Subscribing to %s for motion capture", optiTop.c_str());   
    
}

rigidBody::~rigidBody()
{
    ROS_INFO("Shutting down rigid body %s", tag.c_str());
}

bool rigidBody::getControllable()
{
    return this->controllable;
}

std::string rigidBody::getName()
{
    return this->tag;
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
    desPos.position.x = pos.x;
    desPos.position.y = pos.y;
    desPos.position.z = pos.z;
    commandDuration = duration;
    ROS_INFO("Set z: %f", pos.z);
    // currently do not manage yaw 
    desPos.orientation = currPos.orientation;
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
    geometry_msgs::PoseStamped lastPos = motionCapture.front();
    motionCapture.erase(motionCapture.begin());
    geometry_msgs::PoseStamped firstPos = motionCapture.front();
    currVel = mdp_conversions::calcVel(lastPos,firstPos);
    // ROS_INFO("%s linear velocity [x: %f,y: %f,z: %f]", tag.c_str(), currVel.linear.x, currVel.linear.y, currVel.linear.z);
}

void rigidBody::addMotionCapture(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    motionCapture.push_back(*msg);
    if (motionCapture.size() >= 2){calcVel();}
    currPos = motionCapture.front().pose;
    if (currPos.position.z < 0.05)
    {
        // @TODO: set offsets
        // pitch roll and yaw offset difference between 
        // optitrack and device
    }
    external_pose.publish(msg);
}

geometry_msgs::PoseStamped rigidBody::getMotionCapture()
{
    return motionCapture.front();
}

void rigidBody::update(std::vector<rigidBody*>& rigidBodies)
{
    wrapperControlLoop();

    // direction
    geometry_msgs::Vector3 direction;

    // x change translates to roll
    direction.x = desPos.position.x - currPos.position.x;
    
    // y change translates to pitch
    direction.y = desPos.position.y - currPos.position.y;
    
    // z change translates to thrust
    direction.z = desPos.position.z - currPos.position.z;
    // z < 0 reduce thrust
    // z > 0 increase thrust

}