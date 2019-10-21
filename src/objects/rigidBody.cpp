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

void rigidBody::set_state(const std::string& state)
{
    if (this->State != state) {
        ROS_INFO("%s: %s", this->tag.c_str(), state.c_str());
        this->State = state;
        this->StateIsDirty = true;
    }
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
    set_state("MOVING");
    this->onSetPosition(pos, yaw, duration);
    desPos.position.x = pos.x;
    desPos.position.y = pos.y;
    desPos.position.z = pos.z;
    commandDuration = duration;
    ROS_INFO("Set z: %f", pos.z);
    // @TODO: Orientation Data
    resetTimeout(duration);
}

returnVel rigidBody::getDesVel()
{
    return {desVel.linear, desVel.angular.z, commandDuration};
}

void rigidBody::setDesVel(geometry_msgs::Vector3 vel, float yawRate, float duration)
{
    set_state("MOVING");
    desVel.linear = vel;
    desVel.angular.z = yawRate;
    commandDuration = duration;
    resetTimeout(duration);
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
    if (ros::Time::now().toSec() >= nextTimeoutGen) {
        if (State == "LANDING" || State == "LANDED") {
            set_state("LANDED");
            resetTimeout(100);
        } else {
            if (State != "IDLE") {
                /* Go to hover */
                geometry_msgs::Vector3 currPosVec;
                currPosVec.x = currPos.position.x;
                currPosVec.y = currPos.position.y;
                currPosVec.z = 1;
                setDesPos(currPosVec, 0.0, TIMEOUT_HOVER + 1.0);
                set_state("IDLE");
                timeoutStageOne = false;
                nextTimeoutGen = ros::Time::now().toSec() + TIMEOUT_HOVER;
            } else {
                /* land drone because timeout */
                ROS_WARN("Timeout stage 2");
                this->land();
            }
        }
    }

    this->onUpdate();
}

void rigidBody::emergency()
{
    this->set_state("EMERGENCY");
    this->onEmergency();
}

void rigidBody::land()
{
    this->set_state("LANDING");
    this->onLand(5.0f);
    resetTimeout(5.5f);
}

void rigidBody::takeoff(float height, float duration)
{
    this->set_state("TAKING OFF");
    this->onTakeoff(height, duration);
    resetTimeout(duration);
}

void rigidBody::resetTimeout(float timeout)
{
    ROS_INFO("Reset timer to ~%f seconds", timeout);
    timeout = timeout - 0.2f;
    timeout = std::max(timeout, 0.0f);
    nextTimeoutGen = ros::Time::now().toSec() + timeout;
    timeoutStageOne = true;
}