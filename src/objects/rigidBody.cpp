#include "rigidBody.h"
#include "elementConversions.cpp"
#include "../../include/logger.h"

rigidBody::rigidBody(std::string tag, uint32_t id):mySpin(1,&myQueue)
{
    this->tag = tag;
    this->NumericID = id;
    this->controllable = true; // drone or obstacle
    this->batteryDying = false;
    this->lastRecievedApiUpdate.msg_type = "";
  
    // look for drone under tag namespace then vrpn output
    std::string motionTopic = "/vrpn_client_node/" + tag + "/pose";
    std::string logTopic = tag + "/log";
    std::string updateTopic = tag + "/update";
    std::string ApiTopic = tag + "/apiUpdate";

    droneHandle = ros::NodeHandle();
    ApiPublisher = droneHandle.advertise<multi_drone_platform::apiUpdate> (ApiTopic, 2);
    droneHandle.setCallbackQueue(&myQueue);

    ApiSubscriber = droneHandle.subscribe(ApiTopic, 2, &rigidBody::apiCallback, this);
    LogPublisher = droneHandle.advertise<multi_drone_platform::droneLog> (logTopic, 20);
    MotionSubscriber = droneHandle.subscribe<geometry_msgs::PoseStamped>(motionTopic, 1,&rigidBody::addMotionCapture, this);
    CurrentPosePublisher = droneHandle.advertise<std_msgs::Float64MultiArray> ("mdp/drone_" + std::to_string(NumericID) + "/CurrentPose", 1);
    DesiredPosePublisher = droneHandle.advertise<std_msgs::Float64MultiArray> ("mdp/drone_" + std::to_string(NumericID) + "/DesiredPose", 1);

    this->log(logger::INFO, "Subscribing to motion topic: " + motionTopic);
    this->log(logger::INFO, "Subscrbing to API topic: " + ApiTopic);
    this->log(logger::INFO, "Publishing log data to: " + logTopic);
    this->log(logger::INFO, "Publishing updates to: " + updateTopic);

    // 1000 seconds on ground before timeout engaged 
    set_state("LANDED");
    resetTimeout(1000.0f);
}

rigidBody::~rigidBody()
{
    this->log(logger::INFO, "Shutting down...");
}

void rigidBody::set_state(const std::string& state)
{
    this->log(logger::INFO, "Setting state to " + state);
    this->State = state;
    droneHandle.setParam("mdp/drone_" + std::to_string(this->NumericID) + "/state", state);
}

bool rigidBody::getControllable()
{
    return this->controllable;
}

std::string rigidBody::getName()
{
    return this->tag;
}

float rigidBody::getYaw(geometry_msgs::Pose& pos)
{
    return mdp_conversions::toEuler(pos.orientation).Yaw;
}

geometry_msgs::Vector3 rigidBody::predictCurrentPosition()
{
    double time_since_mocap_update = ros::Time::now().toSec() - lastUpdate.toSec();
    if (time_since_mocap_update < 0.0) {
        ROS_WARN("time since update returning less than 0");
        time_since_mocap_update = 0.0;
    }

    geometry_msgs::Vector3 pos = mdp_conversions::point_to_vector3(CurrentPose.position);
    pos.x += (CurrentVelocity.linear.x * time_since_mocap_update);
    pos.y += (CurrentVelocity.linear.y * time_since_mocap_update);
    pos.z += (CurrentVelocity.linear.z * time_since_mocap_update);

    return pos;
}

double rigidBody::predictCurrentYaw()
{
    double time_since_mocap_update = ros::Time::now().toSec() - lastUpdate.toSec();
    if (time_since_mocap_update < 0.0) {
        ROS_WARN("time since update returning less than 0");
        time_since_mocap_update = 0.0;
    }

    double yaw = getYaw(CurrentPose);
    yaw += (CurrentVelocity.angular.z * time_since_mocap_update);

    return yaw;
}

double vec3Distance(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b)
{
    double dist = 0.0;
    dist += std::abs(a.x - b.x);
    dist += std::abs(a.y - b.y);
    dist += std::abs(a.z - b.z);
    return dist;
}

void rigidBody::setDesPos(geometry_msgs::Vector3 pos, float yaw,
float duration, bool relativeXY, bool relativeZ)
{
    auto current_position = predictCurrentPosition();
    // ROS_INFO("Current:");
    // ROS_INFO("x: %f, y: %f, z: %f", CurrentPosition.position.x, CurrentPosition.position.y, CurrentPosition.position.z);
    // ROS_INFO("Relative: %d", relative);
    // manage relative x, y values 

    /* get z values in terms of x,y values (synchronise relativity) */
    if (relativeZ && !relativeXY) {
        pos.z = pos.z + current_position.z;
    }
    if (!relativeZ && relativeXY) {
        pos.z = pos.z - current_position.z;
    }

    /* Simple static safeguarding */
    struct {
        std::array<double, 2> x = {{-1.60, 0.95}};
        std::array<double, 2> y = {{-1.30, 1.30}};
        std::array<double, 2> z = {{ 0.10, 1.80}};
    } StaticSafeguarding;

    if (relativeXY) {
        /* lowest pos value */
        pos.x = std::max(StaticSafeguarding.x[0] - current_position.x, pos.x);
        pos.y = std::max(StaticSafeguarding.y[0] - current_position.y, pos.y);
        pos.z = std::max(StaticSafeguarding.z[0] - current_position.z, pos.z);

        /* highest pos value */
        pos.x = std::min(StaticSafeguarding.x[1] - current_position.x, pos.x);
        pos.y = std::min(StaticSafeguarding.y[1] - current_position.y, pos.y);
        pos.z = std::min(StaticSafeguarding.z[1] - current_position.z, pos.z);
    } else {
        /* both */
        pos.x = std::min(std::max(StaticSafeguarding.x[0], pos.x), StaticSafeguarding.x[1]);
        pos.y = std::min(std::max(StaticSafeguarding.y[0], pos.y), StaticSafeguarding.y[1]);
        pos.z = std::min(std::max(StaticSafeguarding.z[0], pos.z), StaticSafeguarding.z[1]);
    }

    geometry_msgs::Vector3 abs_pos = pos;
    if (relativeXY) {
        abs_pos.x += this->CurrentPose.position.x;
        abs_pos.y += this->CurrentPose.position.y;
    }
    if (relativeZ) {
        abs_pos.z += this->CurrentPose.position.z;
    }

    this->DesiredPose.position.x = abs_pos.x;
    this->DesiredPose.position.y = abs_pos.y;
    this->DesiredPose.position.z = abs_pos.z;
    // @TODO: need to manage orientation

    this->log(logger::DEBUG, "DesPos: [" + std::to_string(DesiredPose.position.x) + ", " + std::to_string(DesiredPose.position.y) +
    ", " + std::to_string(DesiredPose.position.z) + "] Dur: " + std::to_string(duration));

    this->onSetPosition(pos, yaw, duration, relativeXY);

    resetTimeout(duration);
}

void rigidBody::setDesVel(geometry_msgs::Vector3 vel, float yawRate, 
float duration, bool relativeXY, bool relativeZ)
{
    // @TODO: velocity based safeguarding

    // onVelocity command
    this->onSetVelocity(vel, yawRate, duration, relativeXY);

    this->resetTimeout(duration);
}

bool rigidBody::isMsgSignificantlyDifferent(multi_drone_platform::apiUpdate msg)
{
    double time_between_common_msgs = 0.5;
    double distance_between_common_msgs = 0.1;
    double time_between_the_two_msgs = ros::Time::now().toSec() - timeOfLastApiUpdate.toSec();
    if (time_between_the_two_msgs > time_between_common_msgs) return true; // there has been significant time between msgs
    if (msg.msg_type != lastRecievedApiUpdate.msg_type) return true;        // they are not the same msg type
    if (msg.relativeXY != lastRecievedApiUpdate.relativeXY) return true;    // they do not have the same xy relative
    if (msg.relativeZ != lastRecievedApiUpdate.relativeZ) return true;      // they do not have the same z relative
    if (std::abs(msg.duration - (lastRecievedApiUpdate.duration - time_between_the_two_msgs)) > 0.5) return true; // relative durations are significantly different
    if (vec3Distance(msg.posvel, lastRecievedApiUpdate.posvel) > distance_between_common_msgs) return true; // they are significantly different in destinations
    if (std::abs(msg.yawVal - lastRecievedApiUpdate.yawVal) > 10.0) return true; // they have significantly different yawvals
    return false;   // else they are pretty much the same message
}

geometry_msgs::Vector3 rigidBody::getHomePos()
{
    return HomePosition;
}   

void rigidBody::setHomePos(geometry_msgs::Vector3 pos, bool relative)
{
    // z coord does not matter for home position, will be set in each case
    if (relative)
    {
        HomePosition.x = CurrentPose.position.x + pos.x;
        HomePosition.y = CurrentPose.position.y + pos.y;
    }
    else
    {
        HomePosition.x = pos.x;
        HomePosition.y = pos.y;
    }
    HomePosition.z = 0.0f;
}

void rigidBody::calculateVelocity()
{
    geometry_msgs::PoseStamped lastPos = motionCapture.front();
    motionCapture.erase(motionCapture.begin());
    geometry_msgs::PoseStamped firstPos = motionCapture.front();
    CurrentVelocity = mdp_conversions::calcVel(firstPos,lastPos);
    // ROS_INFO("%s linear velocity [x: %f,y: %f,z: %f]", tag.c_str(), currVel.linear.x, currVel.linear.y, currVel.linear.z);
}

void rigidBody::addMotionCapture(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if(motionCapture.size() == 0){
        HomePosition.x = msg->pose.position.x;
        HomePosition.y = msg->pose.position.y;
        HomePosition.z = msg->pose.position.z;
        
        std::string homePosLog = "HOME POS: [" + std::to_string(HomePosition.x) + ", " 
        + std::to_string(HomePosition.y) + ", " + std::to_string(HomePosition.z) + "]";
        this->log(logger::INFO, homePosLog);
    }
    
    motionCapture.push_back(*msg);
    if (motionCapture.size() >= 2){this->calculateVelocity();}
    CurrentPose = motionCapture.front().pose;
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
                this->log(logger::DEBUG, "Timeout stage 1");
                this->log(logger::WARN, "Timeout hover");
                this->hover(TIMEOUT_HOVER);
                this->set_state("IDLE");
            } else {
                /* land drone because timeout */
                this->log(logger::DEBUG, "Timeout stage 2");
                this->land();
            }
        }
    }
    if (State == "IDLE")
    {
        handleCommand();
    }

    /* publish desired pose and velocity */
    std_msgs::Float64MultiArray Arr;
    Arr.layout.data_offset = 0;
    Arr.data.resize(8);
    Arr.data[0] = this->DesiredPose.position.x;
    Arr.data[1] = this->DesiredPose.position.y;
    Arr.data[2] = this->DesiredPose.position.z;
    Arr.data[3] = getYaw(this->DesiredPose);
    Arr.data[4] = this->DesiredVelocity.linear.x;
    Arr.data[5] = this->DesiredVelocity.linear.y;
    Arr.data[6] = this->DesiredVelocity.linear.z;
    Arr.data[7] = this->DesiredVelocity.angular.y;  // @FIX, is this yawrate?
    DesiredPosePublisher.publish(Arr);

    /* publish current pose and velocity */
    Arr.data[0] = this->CurrentPose.position.x;
    Arr.data[1] = this->CurrentPose.position.y;
    Arr.data[2] = this->CurrentPose.position.z;
    Arr.data[3] = getYaw(this->CurrentPose);
    Arr.data[4] = this->CurrentVelocity.linear.x;
    Arr.data[5] = this->CurrentVelocity.linear.y;
    Arr.data[6] = this->CurrentVelocity.linear.z;
    Arr.data[7] = this->CurrentVelocity.angular.y;  // @FIX, is this yawrate?
    CurrentPosePublisher.publish(Arr);

    this->onUpdate();
}

void rigidBody::apiCallback(const multi_drone_platform::apiUpdate& msg)
{
    if(!batteryDying)
    {
        // ROS_INFO("%s recieved msg %s", tag.c_str(),msg.msg_type.c_str());
        // std::string commandInfo = "Recieved msg " + msg.msg_type;
        // this->postLog(0, commandInfo);  
        this->CommandQueue.clear();
        this->CommandQueue.push_back(msg);
    }
    else
    {
        this->log(logger::ERROR, "Battery Timeout");
    }
    handleCommand();
}

void rigidBody::handleCommand(){
    this->log(logger::INFO, "State: " + State);
    geometry_msgs::Vector3 noMove;
    noMove.x = noMove.y = noMove.z = 0.0f;
    multi_drone_platform::apiUpdate landMsg;
    landMsg.msg_type = "LAND";
    landMsg.duration = 2.0f;

    if (CommandQueue.size() > 0)
    {
        multi_drone_platform::apiUpdate msg = this->CommandQueue.front();
        if (isMsgSignificantlyDifferent(msg)) {
            this->log(logger::INFO, "Sending msg- " + msg.msg_type);
            this->lastRecievedApiUpdate = msg;
            this->timeOfLastApiUpdate = ros::Time::now();
            switch(APIMap[msg.msg_type]) {
                /* VELOCITY */
                case 0:
                    ROS_INFO("V: [%.2f, %.2f, %.2f] rel_Xy: %d, rel_z: %d, dur: %.1f", msg.posvel.x, msg.posvel.y, msg.posvel.z, msg.relativeXY, msg.relativeZ, msg.duration);
                    setDesVel(msg.posvel, msg.yawVal, msg.duration, msg.relativeXY, msg.relativeZ);
                    this->set_state("MOVING");
                    break;
                /* POSITION */
                case 1:
                    ROS_INFO("P: xyz: %.2f %.2f %.2f, rel_Xy: %d, rel_z: %d", msg.posvel.x, msg.posvel.y, msg.posvel.z, msg.relativeXY, msg.relativeZ);
                    setDesPos(msg.posvel, msg.yawVal, msg.duration, msg.relativeXY, msg.relativeZ);
                    this->set_state("MOVING");
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

                    this->hover(msg.duration);
                    break;
                /* EMERGENCY */
                case 5: 
                    emergency();
                    break;
                /* SET_HOME */
                case 6:
                    setHomePos(msg.posvel, msg.relativeXY);
                    break;
                /* GOTO_HOME */
                case 8:
                    ROS_INFO("message duration on goto home %f", msg.duration);
                    setDesPos(HomePosition, msg.yawVal,msg.duration, false, true);
                    this->set_state("MOVING");
                    if (msg.posvel.z <= 0.0f)
                    {
                        enqueueCommand(landMsg);
                    }
                    break;
                default:
                    this->log(logger::WARN, "The API command, " + msg.msg_type + ", is not valid");
                break;
            }
        }
        dequeueCommand();
    }
}

void rigidBody::enqueueCommand(multi_drone_platform::apiUpdate command)
{
    CommandQueue.push_back(command);
}

void rigidBody::dequeueCommand()
{
    // @TODO: the application is seg faulting here for some reason, please fix
    if (CommandQueue.size() > 0) { // this is a hack fix
        auto it = CommandQueue.begin(); 
        CommandQueue.erase(it);
    }
}

void rigidBody::emergency()
{
    this->set_state("EMERGENCY");
    this->onEmergency();
}

void rigidBody::land(float duration)
{
    this->set_state("LANDING");
    this->onLand(duration);
    resetTimeout(duration);
}

void rigidBody::takeoff(float height, float duration) {
    this->set_state("TAKING OFF");
    this->onTakeoff(height, duration);
    resetTimeout(duration);
}

void rigidBody::hover(float duration) {
    this->set_state("HOVER");
    // set the hover point based on current velocity
    // auto pos = this->CurrentPose.position;
    geometry_msgs::Vector3 pos;
    // pos.x += CurrentVelocity.linear.x * 0.3;
    // pos.y += CurrentVelocity.linear.y * 0.3;
    // pos.z += CurrentVelocity.linear.z * 0.3;
    setDesPos(pos, 0.0f, duration, true, true);
}

void rigidBody::resetTimeout(float timeout) {
    this->log(logger::INFO, "Reset timer to " + std::to_string(timeout) + " seconds");
    timeout = timeout - 0.2f;
    timeout = std::max(timeout, 0.0f);
    nextTimeoutGen = ros::Time::now().toSec() + timeout;
    lastCommandSet = ros::Time::now();
}

void rigidBody::log(logger::logType msgType, std::string message) {
    logger::postLog(msgType, this->tag, message, LogPublisher);
}