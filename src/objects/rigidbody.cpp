#include "rigidbody.h"
#include "element_conversions.cpp"


rigidbody::rigidbody(std::string tag, uint32_t id): mySpin(1,&myQueue) {
    this->tag = tag;
    this->numericID = id;
    this->controllable = true; // drone or obstacle
    this->batteryDying = false;
    this->lastRecievedApiUpdate.msgType = "";
  
    // look for drone under tag namespace then vrpn output
    std::string motionTopic = "/vrpn_client_node/" + tag + "/pose";
    std::string logTopic = tag + "/log";
    std::string updateTopic = tag + "/update";
    std::string apiTopic = tag + "/apiUpdate";

    droneHandle = ros::NodeHandle();
    apiPublisher = droneHandle.advertise<multi_drone_platform::api_update> (apiTopic, 2);
    droneHandle.setCallbackQueue(&myQueue);

    apiSubscriber = droneHandle.subscribe(apiTopic, 2, &rigidbody::api_callback, this);
    logPublisher = droneHandle.advertise<multi_drone_platform::log> (logTopic, 20);
    motionSubscriber = droneHandle.subscribe<geometry_msgs::PoseStamped>(motionTopic, 1,&rigidbody::add_motion_capture, this);
    currentPosePublisher = droneHandle.advertise<geometry_msgs::PoseStamped> ("mdp/drone_" + std::to_string(numericID) + "/pose", 1);
    currentVelocityPublisher = droneHandle.advertise<geometry_msgs::TwistStamped> ("mdp/drone_" + std::to_string(numericID) + "/velocity", 1);

    this->log(logger::INFO, "Subscribing to motion topic: " + motionTopic);
    this->log(logger::INFO, "Subscrbing to API topic: " + apiTopic);
    this->log(logger::INFO, "Publishing log data to: " + logTopic);
    this->log(logger::INFO, "Publishing updates to: " + updateTopic);

    // 1000 seconds on ground before timeout engaged 
    set_state("LANDED");
    reset_timeout(1000.0f);
}

rigidbody::~rigidbody() {
    this->log(logger::INFO, "Shutting down...");
}

void rigidbody::set_state(const std::string& state) {
    this->log(logger::INFO, "Setting state to " + state);
    this->state = state;
    droneHandle.setParam("mdp/drone_" + std::to_string(this->numericID) + "/state", state);
}

bool rigidbody::get_controllable() {
    return this->controllable;
}

std::string rigidbody::get_name() {
    return this->tag;
}

geometry_msgs::Vector3 rigidbody::predict_current_position() {
    double timeSinceMoCapUpdate = ros::Time::now().toSec() - lastUpdate.toSec();
    if (timeSinceMoCapUpdate < 0.0) {
        ROS_WARN("time since update returning less than 0");
        timeSinceMoCapUpdate = 0.0;
    }

    geometry_msgs::Vector3 pos = mdp_conversions::point_to_vector3(currentPose.position);
    pos.x += (currentVelocity.linear.x * timeSinceMoCapUpdate);
    pos.y += (currentVelocity.linear.y * timeSinceMoCapUpdate);
    pos.z += (currentVelocity.linear.z * timeSinceMoCapUpdate);

    return pos;
}

double rigidbody::predict_current_yaw() {
    double timeSinceMoCapUpdate = ros::Time::now().toSec() - lastUpdate.toSec();
    if (timeSinceMoCapUpdate < 0.0) {
        ROS_WARN("time since update returning less than 0");
        timeSinceMoCapUpdate = 0.0;
    }

    double yaw = mdp_conversions::get_yaw_from_pose(currentPose);
    yaw += (currentVelocity.angular.z * timeSinceMoCapUpdate);

    return yaw;
}

double rigidbody::vec3_distance(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b)
{
    double dist = 0.0;
    dist += std::abs(a.x - b.x);
    dist += std::abs(a.y - b.y);
    dist += std::abs(a.z - b.z);
    return dist;
}

void rigidbody::set_desired_position(geometry_msgs::Vector3 pos, float yaw,
float duration, bool relativeXY, bool relativeZ) {
    auto currentPosition = predict_current_position();
    // ROS_INFO("Current:");
    // ROS_INFO("x: %f, y: %f, z: %f", CurrentPosition.position.x, CurrentPosition.position.y, CurrentPosition.position.z);
    // ROS_INFO("Relative: %d", relative);
    // manage relative x, y values 

    /* get z values in terms of x,y values (synchronise relativity) */
    if (relativeZ && !relativeXY) {
        pos.z = pos.z + currentPosition.z;
    }
    if (!relativeZ && relativeXY) {
        pos.z = pos.z - currentPosition.z;
    }

    /* Simple static safeguarding */
    struct {
        std::array<double, 2> x = {{-1.60, 0.95}};
        std::array<double, 2> y = {{-1.30, 1.30}};
        std::array<double, 2> z = {{ 0.10, 1.80}};
    } static_safeguarding;

    if (relativeXY) {
        /* lowest pos value */
        pos.x = std::max(static_safeguarding.x[0] - currentPosition.x, pos.x);
        pos.y = std::max(static_safeguarding.y[0] - currentPosition.y, pos.y);
        pos.z = std::max(static_safeguarding.z[0] - currentPosition.z, pos.z);

        /* highest pos value */
        pos.x = std::min(static_safeguarding.x[1] - currentPosition.x, pos.x);
        pos.y = std::min(static_safeguarding.y[1] - currentPosition.y, pos.y);
        pos.z = std::min(static_safeguarding.z[1] - currentPosition.z, pos.z);
    } else {
        /* both */
        pos.x = std::min(std::max(static_safeguarding.x[0], pos.x), static_safeguarding.x[1]);
        pos.y = std::min(std::max(static_safeguarding.y[0], pos.y), static_safeguarding.y[1]);
        pos.z = std::min(std::max(static_safeguarding.z[0], pos.z), static_safeguarding.z[1]);
    }

    geometry_msgs::Vector3 abs_pos = pos;
    if (relativeXY) {
        abs_pos.x += this->currentPose.position.x;
        abs_pos.y += this->currentPose.position.y;
    }
    if (relativeZ) {
        abs_pos.z += this->currentPose.position.z;
    }

    this->desiredPose.position.x = abs_pos.x;
    this->desiredPose.position.y = abs_pos.y;
    this->desiredPose.position.z = abs_pos.z;
    // @TODO: need to manage orientation

    this->log(logger::DEBUG, "DesPos: [" + std::to_string(desiredPose.position.x) + ", " + std::to_string(desiredPose.position.y) +
    ", " + std::to_string(desiredPose.position.z) + "] Dur: " + std::to_string(duration));

    this->on_set_position(pos, yaw, duration, relativeXY);

    reset_timeout(duration);
}

void rigidbody::set_desired_velocity(geometry_msgs::Vector3 vel, float yawRate, 
float duration, bool relativeXY, bool relativeZ) {
    // @TODO: velocity based safeguarding

    // onVelocity command
    this->on_set_velocity(vel, yawRate, duration, relativeXY);

    this->reset_timeout(duration);
}

bool rigidbody::is_msg_different(multi_drone_platform::api_update msg) {
    double timeBetweenCommonMsgs = 0.5;
    double distanceBetweenCommonMsgs = 0.1;
    double timeBetweenTwoMsgs = ros::Time::now().toSec() - timeOfLastApiUpdate.toSec();
    if (timeBetweenTwoMsgs > timeBetweenCommonMsgs) return true; // there has been significant time between msgs
    if (msg.msgType != lastRecievedApiUpdate.msgType) return true;        // they are not the same msg type
    if (msg.relativeXY != lastRecievedApiUpdate.relativeXY) return true;    // they do not have the same xy relative
    if (msg.relativeZ != lastRecievedApiUpdate.relativeZ) return true;      // they do not have the same z relative
    if (std::abs(msg.duration - (lastRecievedApiUpdate.duration - timeBetweenTwoMsgs)) > 0.5) return true; // relative durations are significantly different
    if (vec3_distance(msg.posVel, lastRecievedApiUpdate.posVel) > distanceBetweenCommonMsgs) return true; // they are significantly different in destinations
    if (std::abs(msg.yawVal - lastRecievedApiUpdate.yawVal) > 10.0) return true; // they have significantly different yawvals
    return false;   // else they are pretty much the same message
}

geometry_msgs::Vector3 rigidbody::get_home_coordinates() {
    return homePosition;
}   

void rigidbody::set_home_coordiates(geometry_msgs::Vector3 pos, bool relative) {
    // z coord does not matter for home position, will be set in each case
    if (relative) {
        homePosition.x = currentPose.position.x + pos.x;
        homePosition.y = currentPose.position.y + pos.y;
    }
    else {
        homePosition.x = pos.x;
        homePosition.y = pos.y;
    }
    homePosition.z = 0.0f;
}

void rigidbody::calculate_velocity()
{
    geometry_msgs::PoseStamped lastPos = motionCapture.front();
    motionCapture.erase(motionCapture.begin());
    geometry_msgs::PoseStamped firstPos = motionCapture.front();
    currentVelocity = mdp_conversions::calc_vel(firstPos,lastPos);
    // ROS_INFO("%s linear velocity [x: %f,y: %f,z: %f]", tag.c_str(), currVel.linear.x, currVel.linear.y, currVel.linear.z);
}

void rigidbody::add_motion_capture(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if(motionCapture.size() == 0) {
        homePosition.x = msg->pose.position.x;
        homePosition.y = msg->pose.position.y;
        homePosition.z = msg->pose.position.z;
        
        std::string homePosLog = "HOME POS: [" + std::to_string(homePosition.x) + ", " 
        + std::to_string(homePosition.y) + ", " + std::to_string(homePosition.z) + "]";
        this->log(logger::INFO, homePosLog);
    }
    
    motionCapture.push_back(*msg);
    if (motionCapture.size() >= 2){this->calculate_velocity();}
    currentPose = motionCapture.front().pose;
    // ROS_INFO("Current Position: x: %f, y: %f, z: %f",currPos.position.x, currPos.position.y, currPos.position.z);
    // @TODO: Orientation implementation

    this->publish_physical_state();

    this->lastUpdate = ros::Time::now();
    this->on_motion_capture(msg);
}

geometry_msgs::PoseStamped rigidbody::get_motion_capture() {
    return motionCapture.front();
}

void rigidbody::publish_physical_state() const {
    currentPosePublisher.publish(motionCapture.front());

    geometry_msgs::TwistStamped stampedVel = {};
    stampedVel.header.stamp = ros::Time::now();
    stampedVel.twist = this->currentVelocity;
    currentVelocityPublisher.publish(stampedVel);
}

void rigidbody::update(std::vector<rigidbody*>& rigidbodies) {
    if (ros::Time::now().toSec() >= nextTimeoutGen) {
        if (state == "LANDING" || state == "LANDED") {
            set_state("LANDED");
            reset_timeout(100);
        } 
        else {
            if (state != "IDLE") {
                /* Go to hover */
                this->log(logger::DEBUG, "Timeout stage 1");
                this->log(logger::WARN, "Timeout hover");
                this->hover(TIMEOUT_HOVER);
                this->set_state("IDLE");
            } 
            else {
                /* land drone because timeout */
                this->log(logger::DEBUG, "Timeout stage 2");
                this->land();
            }
        }
    }
    if (state == "IDLE") {
        handle_command();
    }

    this->on_update();
}

void rigidbody::api_callback(const multi_drone_platform::api_update& msg) {
    if(!batteryDying) {
        // ROS_INFO("%s recieved msg %s", tag.c_str(),msg.msg_type.c_str());
        // std::string commandInfo = "Recieved msg " + msg.msg_type;
        // this->postLog(0, commandInfo);  
        this->commandQueue.clear();
        this->commandQueue.push_back(msg);
    }
    else {
        this->log(logger::ERROR, "Battery Timeout");
    }
    handle_command();
}

void rigidbody::handle_command() {
    this->log(logger::INFO, "State: " + state);
    geometry_msgs::Vector3 noMove;
    noMove.x = noMove.y = noMove.z = 0.0f;
    multi_drone_platform::api_update landMsg;
    landMsg.msgType = "LAND";
    landMsg.duration = 2.0f;

    if (commandQueue.size() > 0) {
        multi_drone_platform::api_update msg = this->commandQueue.front();
        if (is_msg_different(msg)) {
            this->log(logger::INFO, "Sending msg- " + msg.msgType);
            this->lastRecievedApiUpdate = msg;
            this->timeOfLastApiUpdate = ros::Time::now();
            switch(apiMap[msg.msgType]) {
                /* VELOCITY */
                case 0:
                    ROS_INFO("V: [%.2f, %.2f, %.2f] rel_Xy: %d, rel_z: %d, dur: %.1f", msg.posVel.x, msg.posVel.y, msg.posVel.z, msg.relativeXY, msg.relativeZ, msg.duration);
                    set_desired_velocity(msg.posVel, msg.yawVal, msg.duration, msg.relativeXY, msg.relativeZ);
                    this->set_state("MOVING");
                    break;
                /* POSITION */
                case 1:
                    ROS_INFO("P: xyz: %.2f %.2f %.2f, rel_Xy: %d, rel_z: %d", msg.posVel.x, msg.posVel.y, msg.posVel.z, msg.relativeXY, msg.relativeZ);
                    set_desired_position(msg.posVel, msg.yawVal, msg.duration, msg.relativeXY, msg.relativeZ);
                    this->set_state("MOVING");
                    break;
                /* TAKEOFF */
                case 2:
                    ROS_INFO("Height: %f", msg.posVel.z);
                    takeoff(msg.posVel.z, msg.duration);
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
                    set_home_coordiates(msg.posVel, msg.relativeXY);
                    break;
                /* GOTO_HOME */
                case 8:
                    ROS_INFO("message duration on goto home %f", msg.duration);
                    set_desired_position(homePosition, msg.yawVal,msg.duration, false, true);
                    this->set_state("MOVING");
                    if (msg.posVel.z <= 0.0f)
                    {
                        enqueue_command(landMsg);
                    }
                    break;
                default:
                    this->log(logger::WARN, "The API command, " + msg.msgType + ", is not valid");
                break;
            }
        }
        dequeue_command();
    }
}

void rigidbody::enqueue_command(multi_drone_platform::api_update command) {
    commandQueue.push_back(command);
}

void rigidbody::dequeue_command() {
    // @TODO: the application is seg faulting here for some reason, please fix
    if (commandQueue.size() > 0) { // this is a hack fix
        auto it = commandQueue.begin(); 
        commandQueue.erase(it);
    }
}

void rigidbody::emergency() {
    this->set_state("EMERGENCY");
    this->on_emergency();
}

void rigidbody::land(float duration) {
    this->set_state("LANDING");
    this->on_land(duration);
    reset_timeout(duration);
}

void rigidbody::takeoff(float height, float duration) {
    this->set_state("TAKING OFF");
    this->on_takeoff(height, duration);
    reset_timeout(duration);
}

void rigidbody::hover(float duration) {
    this->set_state("HOVER");
    // set the hover point based on current velocity
    // auto pos = this->CurrentPose.position;
    geometry_msgs::Vector3 pos;
    // pos.x += CurrentVelocity.linear.x * 0.3;
    // pos.y += CurrentVelocity.linear.y * 0.3;
    // pos.z += CurrentVelocity.linear.z * 0.3;
    set_desired_position(pos, 0.0f, duration, true, true);
}

void rigidbody::reset_timeout(float timeout) {
    this->log(logger::INFO, "Reset timer to " + std::to_string(timeout) + " seconds");
    timeout = timeout - 0.2f;
    timeout = std::max(timeout, 0.0f);
    nextTimeoutGen = ros::Time::now().toSec() + timeout;
    lastCommandSet = ros::Time::now();
}

void rigidbody::log(logger::log_type msgType, std::string message) {
    logger::post_log(msgType, this->tag, message, logPublisher);
}
