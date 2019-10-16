#include "rigidBody.h"
#include "elementConversions.cpp"


rigidBody::rigidBody(std::string tag, bool controllable)
{
    this->tag = tag;
    // drone or obstacle
    this->controllable = controllable;

    // look for drone under tag namespace then vrpn output
    std::string optiTop = "/vrpn_client_node/" + tag + "/pose";

    // private function initialise
    // set home to be current pos
    // first time, set home to current

    resetTimeout(1000.0f);
    
    commandDuration = 0.0f;

    droneHandle = ros::NodeHandle();
    
    motionSub = droneHandle.subscribe<geometry_msgs::PoseStamped>(optiTop, 10,&rigidBody::addMotionCapture, this);
    
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
    if (!noMoreCommands) {
        this->onSetPosition(pos, yaw, duration);
        desPos.position.x = pos.x;
        desPos.position.y = pos.y;
        desPos.position.z = pos.z;
        commandDuration = duration;
        ROS_INFO("Set z: %f", pos.z);
        // @TODO: Orientation Data
        // resetTimeout(duration - 0.1);
    }
}

returnVel rigidBody::getDesVel()
{
    return {desVel.linear, desVel.angular.z, commandDuration};
}

void rigidBody::setDesVel(geometry_msgs::Vector3 vel, float yawRate, float duration)
{
    if (!noMoreCommands) {
        desVel.linear = vel;
        desVel.angular.z = yawRate;
        commandDuration = duration;
        resetTimeout();
    }
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
    if(motionCapture.size() == 0){
        homePos.x = msg->pose.position.x;
        homePos.y = msg->pose.position.y;
        homePos.z = msg->pose.position.z;

        ROS_INFO("HOME POS: [%f, %f, %f]", homePos.x, homePos.y, homePos.z);
     }
    motionCapture.push_back(*msg);
    if (motionCapture.size() >= 2){calcVel();}
    currPos = motionCapture.front().pose;
    // ROS_INFO("Current Position: x: %f, y: %f, z: %f",currPos.position.x, currPos.position.y, currPos.position.z);
    // @TODO: Orientation implementation
    this->onMotionCapture(msg);
}

geometry_msgs::PoseStamped rigidBody::getMotionCapture()
{
    return motionCapture.front();
}

void rigidBody::update(std::vector<rigidBody*>& rigidBodies)
{
    if (!noMoreCommands) {
        if (ros::Time::now().toSec() >= nextTimeoutGen) {
            if (timeoutStageOne) {
                /* Go to hover */
                ROS_WARN("Timeout stage 1");
                geometry_msgs::Vector3 currPosVec;
                currPosVec.x = currPos.position.x;
                currPosVec.y = currPos.position.y;
                currPosVec.z = 1;
                setDesPos(currPosVec, 0.0, TIMEOUT_HOVER + 1.0);
                timeoutStageOne = false;
                nextTimeoutGen = ros::Time::now().toSec() + TIMEOUT_HOVER;
            } else {
                /* land drone because timeout */
                ROS_WARN("Timeout stage 2");
                this->land();
            }
        }

        this->onUpdate();

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
}

void rigidBody::emergency()
{
    this->onEmergency();
}

void rigidBody::land()
{
    this->onLand();
    noMoreCommands = true;
}

void rigidBody::takeoff(float height)
{
    if (!noMoreCommands) {
        this->onTakeoff(height);
        resetTimeout(1.0f);
    }
}

void rigidBody::resetTimeout(float timeout)
{
    nextTimeoutGen = ros::Time::now().toSec() + timeout;
    timeoutStageOne = true;
    ROS_INFO("Reset timer to %f seconds", timeout);
}