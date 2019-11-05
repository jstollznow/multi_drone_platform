#include "rigidBody.h"
#include "elementConversions.cpp"



rigidBody::rigidBody(std::string tag):mySpin(1,&myQueue)
{
    this->tag = tag;
    // drone or obstacle
    this->controllable = true;

    this->batteryDying = false;

    // look for drone under tag namespace then vrpn output
    std::string optiTop = "/vrpn_client_node/" + tag + "/pose";

    

    // 1000 seconds on ground before timeout engaged 
    set_state("LANDED");
    resetTimeout(1000.0f);

    droneHandle = ros::NodeHandle();

    std::string ApiTopic = tag + "/apiUpdate";
    ApiPublisher = droneHandle.advertise<multi_drone_platform::apiUpdate> (ApiTopic, 20);
    ApiSubscriber = droneHandle.subscribe(ApiTopic, 20, &rigidBody::apiCallback, this);

    droneHandle.setCallbackQueue(&myQueue);
    
    motionSub = droneHandle.subscribe<geometry_msgs::PoseStamped>(optiTop, 10,&rigidBody::addMotionCapture, this);
    
    ROS_INFO("Subscribing to %s for motion capture", optiTop.c_str());   
}

rigidBody::~rigidBody()
{
    ROS_INFO("Shutting down rigid body %s", tag.c_str());
}

void rigidBody::setID(uint32_t id)
{
    this->id = id;
}

void rigidBody::set_state(const std::string& state)
{
    ROS_INFO("Setting state to %s", state.c_str());
    this->State = state;
    droneHandle.setParam("mdp/drone_" + std::to_string(id) + "/state", state);
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
    return {lastUpdate, vec3PosConvert(currPos), getYaw(currPos)};
}

returnVel rigidBody::getCurrVel()
{
    float duration = 0;
    return {lastUpdate, currVel.linear, currVel.angular.z};
}

returnPos rigidBody::getDesPos()
{
    return {lastUpdate, vec3PosConvert(desPos), getYaw(desPos)};
}

void rigidBody::setDesPos(geometry_msgs::Vector3 pos, float yaw,
float duration, bool relative, bool constHeight)
{
    set_state("MOVING");
    ROS_INFO("Current:");
    ROS_INFO("x: %f, y: %f, z: %f", currPos.position.x, currPos.position.y, currPos.position.z);
    ROS_INFO("Relative: %d", relative);
    // manage relative x, y values 
    if (relative == true)
    {
        desPos.position.x = (double)currPos.position.x + (double)pos.x;
        desPos.position.y = (double)currPos.position.y + (double)pos.y;
    }
    else
    {
        // ROS_INFO("Why?");
        desPos.position.x = pos.x;
        desPos.position.y = pos.y;
    }

    // manage relative z values 
    if(constHeight)
    {
        desPos.position.z = (double)currPos.position.z + (double)pos.z;
    }
    else
    {
        desPos.position.z = pos.z;
    }
    // static safeguarding
    desPos.position.x = std::min(std::max(-1.6f, (float)desPos.position.x), 0.95f); //2.45
    desPos.position.y = std::min(std::max(-1.30f, (float)desPos.position.y), 1.30f);
    desPos.position.z = std::min(std::max(0.10f, (float)desPos.position.z), 1.8f);

    // @TODO: need to manage orientation
    ROS_INFO("Desired:");
    ROS_INFO("x: %f, y: %f, z: %f", desPos.position.x, desPos.position.y, desPos.position.z);
    ROS_INFO("Duration: %f", duration);
    this->onSetPosition(desPos, yaw, duration);
    resetTimeout(duration);
}

returnVel rigidBody::getDesVel()
{
    return {lastCommandSet, desVel.linear, desVel.angular.z};
}

void rigidBody::setDesVel(geometry_msgs::Vector3 vel, float yawRate, 
float duration, bool relative, bool constHeight)
{
    set_state("MOVING");
    // manage relative x, y values 
    if (relative)
    {
        desVel.linear.x = (double)currVel.linear.x + (double)vel.x;
        desVel.linear.y = (double)currVel.linear.y + (double)vel.y;
    }
    else
    {
        desVel.linear.x = vel.x;
        desVel.linear.y = vel.y;
    }

    // manage relative z values 
    if(constHeight)
    {
        desVel.linear.z = (double)currVel.linear.z + (double)vel.z;
    }
    else
    {
        desVel.linear.z = vel.z;
    }

    desVel.angular.z = yawRate;
    
    // onVelocity command
    this->onSetVelocity(desVel, duration);

    this->resetTimeout(duration);
}

geometry_msgs::Vector3 rigidBody::getHomePos()
{
    return homePos;
}   

void rigidBody::setHomePos(geometry_msgs::Vector3 pos, bool relative)
{
    // z coord does not matter for home position, will be set in each case
    if (relative)
    {
        homePos.x = currPos.position.x + pos.x;
        homePos.y = currPos.position.y + pos.y;
    }
    else
    {
        homePos.x = pos.x;
        homePos.y = pos.y;
    }
    homePos.z = 0.0f;
}

void rigidBody::calcVel()
{
    geometry_msgs::PoseStamped lastPos = motionCapture.front();
    motionCapture.erase(motionCapture.begin());
    geometry_msgs::PoseStamped firstPos = motionCapture.front();
    currVel = mdp_conversions::calcVel(firstPos,lastPos);
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

    this->lastUpdate = ros::Time::now();
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
                ROS_WARN("Timeout stage 1");

                geometry_msgs::Vector3 dummyVec;
                dummyVec.x = 0.0f;
                dummyVec.y = 0.0f;
                dummyVec.z = 0.0f;
                ROS_INFO("Timeout Hover");
                setDesPos(dummyVec, 0.0f, TIMEOUT_HOVER, true, true);
                set_state("IDLE");
                //nextTimeoutGen = ros::Time::now().toSec() + TIMEOUT_HOVER;
            } else {
                /* land drone because timeout */
                ROS_WARN("Timeout stage 2");
                this->land();
            }
        }
    }
    if (State == "IDLE")
    {
        handleCommand();
    }

    this->onUpdate();
}

void rigidBody::apiCallback(const multi_drone_platform::apiUpdate& msg)
{
    if(!batteryDying)
    {
        ROS_INFO("%s recieved msg %s", tag.c_str(),msg.msg_type.c_str());
        this->commandQueue.clear();
        this->commandQueue.push_back(msg);   
    }
    else
    {
        ROS_WARN("Battery Timeout");
    }
    handleCommand();
}

void rigidBody::handleCommand(){
    //ROS_INFO("%s: %s",tag.c_str(),State.c_str());
    geometry_msgs::Vector3 noMove;
    noMove.x = noMove.y = noMove.z = 0.0f;
    multi_drone_platform::apiUpdate landMsg;
    landMsg.msg_type = "LAND";
    landMsg.duration = 2.0f;
    if (commandQueue.size() > 0)
    {
        multi_drone_platform::apiUpdate msg = this->commandQueue.front();
        switch(APIMap[msg.msg_type]) {
            /* VELOCITY */
            case 0:
                ROS_INFO("V: [%.2f, %.2f, %.2f] rel_Xy: %d, rel_z: %d, dur: %.1f", msg.posvel.x, msg.posvel.y, msg.posvel.z, msg.relative, msg.constHeight, msg.duration);
                setDesVel(msg.posvel, msg.yawVal, msg.duration, msg.relative, msg.constHeight);
                break;
            /* POSITION */
            case 1:
                ROS_INFO("POS: xyz: %.2f %.2f %.2f, rel_Xy: %d, rel_z: %d", msg.posvel.x, msg.posvel.y, msg.posvel.z, msg.relative, msg.constHeight);
                setDesPos(msg.posvel, msg.yawVal, msg.duration, msg.relative, msg.constHeight);
                break;
            /* TAKEOFF */
            case 2:
                ROS_INFO("Height: %f", msg.posvel.z);
                takeoff(msg.posvel.z, msg.duration);
                break;
            /* LAND */
            case 3: 
                if (msg.duration != 0.0f){ land(msg.duration); }
                else { land(); }
                break;
            /* HOVER */
            case 4:
                ROS_INFO("message duration on hover %f", msg.duration);
                if (msg.duration == 0.0f) msg.duration = 2.0f;

                setDesPos(noMove, msg.yawVal, msg.duration, true, true);
                break;
            /* EMERGENCY */
            case 5: 
                emergency();
                break;
            /* SET_HOME */
            case 6:
                setHomePos(msg.posvel, msg.relative);
                break;
            /* GOTO_HOME */
            case 8:
                ROS_INFO("message duration on goto home %f", msg.duration);
                setDesPos(homePos, msg.yawVal,msg.duration, false, true);
                if (msg.posvel.z <= 0.0f)
                {
                    enqueueCommand(landMsg);
                }
                break;
            default:
                ROS_ERROR_STREAM("The API command, '" << msg.msg_type << "' is not valid.");
            break;
        }
        dequeueCommand();
    }
    
    
}

void rigidBody::enqueueCommand(multi_drone_platform::apiUpdate command)
{
    commandQueue.push_back(command);
}

void rigidBody::dequeueCommand()
{
    auto it = commandQueue.begin(); 
    commandQueue.erase(it);
}

void rigidBody::emergency()
{
    this->set_state("EMERGENCY");
    this->onEmergency();
}

void rigidBody::land(float duration)
{
    set_state("LANDING");
    this->onLand(duration);
    resetTimeout(duration);
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
    lastCommandSet = ros::Time::now();
}