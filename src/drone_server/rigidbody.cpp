#include <queue>
#include "rigidbody.h"
#include "element_conversions.cpp"

rigidbody::rigidbody(std::string tag, uint32_t id): mySpin(1,&myQueue) {
    this->tag = tag;
    this->numericID = id;
    this->batteryDying = false;
    this->lastRecievedApiUpdate.msgType = "";
  
    // look for drone under tag namespace then vrpn output
    std::string idStr = std::to_string(numericID);

    std::string motionTopic = "/vrpn_client_node/" + tag + "/pose";
    std::string logTopic = "mdp/drone_" + idStr + "/log";
    std::string apiTopic = "mdp/drone_" + idStr + "/apiUpdate";
    std::string batteryTopic = "mdp/drone_" + idStr + "/battery";
    std::string currPoseTopic = "mdp/drone_" + idStr + "/curr_pose";
    std::string desPoseTopic = "mdp/drone_" + idStr + "/des_pose";
    std::string currTwistTopic = "mdp/drone_" + idStr + "/curr_twist";
    std::string desTwistTopic = "mdp/drone_" + idStr + "/des_twist";
    droneHandle = ros::NodeHandle();
    droneHandle.setCallbackQueue(&myQueue);

    apiPublisher = droneHandle.advertise<multi_drone_platform::api_update> (apiTopic, 2);
    apiSubscriber = droneHandle.subscribe(apiTopic, 2, &rigidbody::api_callback, this);
    logPublisher = droneHandle.advertise<multi_drone_platform::log> (logTopic, 20);
    motionSubscriber = droneHandle.subscribe<geometry_msgs::PoseStamped>(motionTopic, 1,&rigidbody::add_motion_capture, this);
    batteryPublisher = droneHandle.advertise<std_msgs::String>(batteryTopic, 1);

    currentPosePublisher = droneHandle.advertise<geometry_msgs::PoseStamped> (currPoseTopic, 1);
    desiredPosePublisher = droneHandle.advertise<geometry_msgs::PoseStamped>(desPoseTopic, 1);

    currentTwistPublisher = droneHandle.advertise<geometry_msgs::TwistStamped> (currTwistTopic, 1);
    desiredTwistPublisher = droneHandle.advertise<geometry_msgs::TwistStamped>(desTwistTopic, 1);

    this->log(logger::INFO, "My id is: " + std::to_string(id));
    this->log(logger::INFO, "Subscribing to motion topic: " + motionTopic);
    this->log(logger::INFO, "Subscrbing to API topic: " + apiTopic);
    this->log(logger::INFO, "Publishing log data to: " + logTopic);
    this->log(logger::INFO, "Publishing current position to: " + currPoseTopic);
    this->log(logger::INFO, "Publishing desired position to: " + desPoseTopic);
    this->log(logger::INFO, "Publishing current velocity to: " + currTwistTopic);
    this->log(logger::INFO, "Publishing desired velocity to: " + desTwistTopic);

    this->set_state(flight_state::LANDED);
}

rigidbody::~rigidbody() {
    this->log(logger::INFO, "Deconstructing...");
    droneHandle.shutdown();
}

void rigidbody::shutdown() {
    /* disable incoming api updates so that go_home cannot be interrupted */
    this->shutdownHasBeenCalled = true;
    this->log(logger::INFO, "Landing drone for shut down");

    /* create and enqueue go to home command */
    multi_drone_platform::api_update msg;
    msg.msgType = "GOTO_HOME";
    msg.yawVal = 0.0;
    msg.posVel.z = 0.0;
    msg.duration = 4.0;
    this->enqueue_command(msg);

    /* immediately handle go home command */
    this->handle_command();
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
    if (this->get_state() == flight_state::LANDED) {
        this->log(logger::WARN, "Go to position called on landed drone, ignoring");
        return;
    }

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
//        std::array<double, 2> x = {{-1.60, 0.95}};
//        std::array<double, 2> y = {{-1.30, 1.30}};
//        std::array<double, 2> z = {{ 0.10, 1.80}};

// for vflie
        std::array<double, 2> x = {{-3.00, 3.00}};
        std::array<double, 2> y = {{-3.00, 3.00}};
        std::array<double, 2> z = {{ 0.10, 3.00}};
    } staticSafeguarding;

    if (relativeXY) {
        /* lowest pos value */
        pos.x = std::max(staticSafeguarding.x[0] - currentPosition.x, pos.x);
        pos.y = std::max(staticSafeguarding.y[0] - currentPosition.y, pos.y);
        pos.z = std::max(staticSafeguarding.z[0] - currentPosition.z, pos.z);

        /* highest pos value */
        pos.x = std::min(staticSafeguarding.x[1] - currentPosition.x, pos.x);
        pos.y = std::min(staticSafeguarding.y[1] - currentPosition.y, pos.y);
        pos.z = std::min(staticSafeguarding.z[1] - currentPosition.z, pos.z);
    } else {
        /* both */
        pos.x = std::min(std::max(staticSafeguarding.x[0], pos.x), staticSafeguarding.x[1]);
        pos.y = std::min(std::max(staticSafeguarding.y[0], pos.y), staticSafeguarding.y[1]);
        pos.z = std::min(std::max(staticSafeguarding.z[0], pos.z), staticSafeguarding.z[1]);
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
    geometry_msgs::PoseStamped desPoseMsg;
    desPoseMsg.pose = desiredPose;
    desPoseMsg.header.stamp = timeOfLastApiUpdate;
    desiredPosePublisher.publish(desPoseMsg);

    /* declare flight state to moving if this is not a hover message */
    if (!(pos.x == 0.0f && pos.y == 0.0f && pos.z == 0.0f && relativeXY && relativeZ)) {
        this->declare_expected_state(flight_state::MOVING);
    }

    this->on_set_position(pos, yaw, duration, relativeXY);
}

void rigidbody::set_desired_velocity(geometry_msgs::Vector3 vel, float yawRate, float duration, bool relativeXY, bool relativeZ) {
    if (this->get_state() == flight_state::LANDED) {
        this->log(logger::WARN, "set velocity called on landed drone, ignoring");
        return;
    }
    // @TODO: velocity based safeguarding

    // onVelocity command

    this->desiredVelocity.linear = vel;
    this->desiredVelocity.angular.z = yawRate;

    geometry_msgs::TwistStamped desTwistMsg;
    desTwistMsg.twist = desiredVelocity;
    desTwistMsg.header.stamp = timeOfLastApiUpdate;

    desiredTwistPublisher.publish(desTwistMsg);

    this->declare_expected_state(flight_state::MOVING);
    this->on_set_velocity(vel, yawRate, duration, relativeXY);
}

bool rigidbody::is_msg_different(multi_drone_platform::api_update msg) {
    double timeBetweenCommonMsgs = 0.5;
    double distanceBetweenCommonMsgs = 0.02;
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
    currentVelocity = mdp_conversions::calc_vel(motionCapture.back(), motionCapture.front());
    // ROS_INFO("%s linear velocity [x: %f,y: %f,z: %f]", tag.c_str(), currVel.linear.x, currVel.linear.y, currVel.linear.z);
}

void rigidbody::add_motion_capture(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    /* if this is the first recieved message, fill motionCapture with homePositions */
    geometry_msgs::PoseStamped motionMsg;

//  convert VRPN data to Z-Up
    motionMsg.header = msg->header;

    motionMsg.pose.position.x = msg->pose.position.y * -1;
    motionMsg.pose.position.y = msg->pose.position.x;
    motionMsg.pose.position.z = msg->pose.position.z;

    motionMsg.pose.orientation.x = msg->pose.orientation.y * -1;
    motionMsg.pose.orientation.y = msg->pose.orientation.x;
    motionMsg.pose.orientation.z = msg->pose.orientation.z;
    motionMsg.pose.orientation.w = msg->pose.orientation.w;

    if(motionCapture.size() == 0) {
        motionCapture.push(motionMsg);
        motionCapture.push(motionMsg);

        homePosition.x = motionMsg.pose.position.x;
        homePosition.y = motionMsg.pose.position.y;
        homePosition.z = motionMsg.pose.position.z;
        
        std::string homePosLog = "HOME POS: [" + std::to_string(homePosition.x) + ", " 
        + std::to_string(homePosition.y) + ", " + std::to_string(homePosition.z) + "]";
        this->log(logger::INFO, homePosLog);
    }

    /* otherwise, erase oldest message and push back newest message */
    motionCapture.pop();
    motionCapture.push(motionMsg);
    this->calculate_velocity();

    currentPose = motionCapture.back().pose;
    // ROS_INFO("Current Position: x: %f, y: %f, z: %f",currPos.position.x, currPos.position.y, currPos.position.z);
    // @TODO: Orientation implementation

    if (ros::Time::now().toSec() > this->declaredStateEndTime) {
        this->update_current_flight_state();
    }
    this->publish_physical_state();

    this->lastUpdate = ros::Time::now();
    this->on_motion_capture(motionMsg);
}

geometry_msgs::PoseStamped rigidbody::get_motion_capture() {
    return motionCapture.back();
}

void rigidbody::publish_physical_state() const {
    currentPosePublisher.publish(motionCapture.back());

    geometry_msgs::TwistStamped stampedVel = {};
    stampedVel.header.stamp = ros::Time::now();
    stampedVel.twist = this->currentVelocity;
    currentTwistPublisher.publish(stampedVel);
}

void rigidbody::update(std::vector<rigidbody*>& rigidbodies) {
    if (this->hoverTimer.has_timed_out()) {
        if (this->hoverTimer.is_stage_timeout()) {
            this->log(logger::WARN, "Timeout stage 2: Landing drone");

            /* send land command through api */
            multi_drone_platform::api_update msg;
            msg.msgType = "LAND";
            msg.duration = 5.0f;
            apiPublisher.publish(msg);
        } else {
            this->do_stage_1_timeout();
        }
    }

    this->on_update();
}

void rigidbody::api_callback(const multi_drone_platform::api_update& msg) {
    if (!shutdownHasBeenCalled) {
        /* if shutdown has been called, then disable all incoming api updates */
        if (!batteryDying) {
            // ROS_INFO("%s recieved msg %s", tag.c_str(),msg.msg_type.c_str());
            // std::string commandInfo = "Recieved msg " + msg.msg_type;
            // this->postLog(0, commandInfo);  
            this->commandQueue.clear();
            this->commandQueue.push_back(msg);
            handle_command();
        } else {
            this->log(logger::ERROR, "Battery Timeout");
            /* shutdown will tell the drone to go to home and land, it will
             * also disable future api updates on this drone */
            this->shutdown();
        }
    }
}

void rigidbody::handle_command() {
    geometry_msgs::Vector3 noMove;
    noMove.x = noMove.y = noMove.z = 0.0f;

    if (commandQueue.size() > 0) {
        this->hoverTimer.close_timer(); // stop timeout 2 from happening as drone has received a message
        bool isGoHomeMessage = false;
        multi_drone_platform::api_update msg = this->commandQueue.front();
        if (is_msg_different(msg)) {
            this->log(logger::INFO, "=> Handling command: " + msg.msgType);
            this->lastRecievedApiUpdate = msg;
            this->timeOfLastApiUpdate = ros::Time::now();
            this->commandEnd = timeOfLastApiUpdate + ros::Duration(msg.duration);
            switch(apiMap[msg.msgType]) {
                /* VELOCITY */
                case 0:
                    ROS_INFO("V: [%.2f, %.2f, %.2f] rel_Xy: %d, rel_z: %d, dur: %.1f", msg.posVel.x, msg.posVel.y, msg.posVel.z, msg.relativeXY, msg.relativeZ, msg.duration);
                    set_desired_velocity(msg.posVel, msg.yawVal, msg.duration, msg.relativeXY, msg.relativeZ);
                    break;
                /* POSITION */
                case 1:
                    ROS_INFO("P: xyz: %.2f %.2f %.2f, rel_Xy: %d, rel_z: %d", msg.posVel.x, msg.posVel.y, msg.posVel.z, msg.relativeXY, msg.relativeZ);
                    set_desired_position(msg.posVel, msg.yawVal, msg.duration, msg.relativeXY, msg.relativeZ);
                    break;
                /* TAKEOFF */
                case 2:
                    ROS_INFO("Height: %f", msg.posVel.z);
                    this->log(logger::INFO, "Duration: " + std::to_string(msg.duration));
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
                    this->hoverTimer.reset_timer(msg.duration - 0.05f);
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
                    go_home(msg.yawVal, msg.duration, msg.posVel.z);
                    isGoHomeMessage = true;
                    break;
                default:
                    this->log(logger::WARN, "The API command, " + msg.msgType + ", is not valid");
                break;
            }
        }
        dequeue_command();

        if (isGoHomeMessage) {
            /* immediately perform set position command enqueued during gohome call */
            handle_command();
        }
    }
}

void rigidbody::enqueue_command(multi_drone_platform::api_update command) {
    commandQueue.push_back(command);
}

void rigidbody::dequeue_command() {
    // @TODO: the application is seg faulting here for some reason, please fix
    // Why is this a vector not a queue? Queues are good for multithreading applications
    // It was originally a queue
    if (commandQueue.size() > 0) { // this is a hack fix
        auto it = commandQueue.begin();
        commandQueue.erase(it);
    }
}

void rigidbody::emergency() {
    this->set_state(flight_state::DELETED);
    this->on_emergency();
}

void rigidbody::land(float duration) {
    /* ignore if drone has already landed, or is already landing */
    if (this->get_state() == flight_state::LANDED) {
        return;
    }

    this->declare_expected_state(flight_state::MOVING);
    this->on_land(duration);
}

void rigidbody::takeoff(float height, float duration) {
    if (this->get_state() != flight_state::LANDED) {
        this->log(logger::WARN, "takeoff called when already in flight, ignoring");
        return;
    }

    this->declare_expected_state(flight_state::MOVING);
    this->on_takeoff(height, duration);
}

void rigidbody::hover(float duration) {
    if (this->get_state() == flight_state::LANDED) {
        this->log(logger::WARN, "hover called on landed drone, ignoring");
        return;
    }

    // set the hover point based on current velocity
    // auto pos = this->CurrentPose.position;
    geometry_msgs::Vector3 pos{};
    // pos.x += CurrentVelocity.linear.x * 0.3;
    // pos.y += CurrentVelocity.linear.y * 0.3;
    // pos.z += CurrentVelocity.linear.z * 0.3;
    this->log_coord(logger::DEBUG, "Position at hover request: ", mdp_conversions::point_to_vector3(currentPose.position));
    set_desired_position(pos, 0.0f, duration, true, true);
    this->declare_expected_state(flight_state::HOVERING);
}

void rigidbody::go_home(float yaw, float duration, float height) {
    /* enqueue goto command */
    multi_drone_platform::api_update goMsg;
    goMsg.msgType = "POSITION";
    goMsg.duration = duration;
    goMsg.relativeXY = false;
    goMsg.relativeZ = true;
    goMsg.yawVal = yaw;
    goMsg.posVel = homePosition;
    goMsg.posVel.z = 0.0;
    enqueue_command(goMsg);

    /* enqueue land msg if necessary */
    if (height < 0.1f) {
        multi_drone_platform::api_update landMsg;
        landMsg.msgType = "LAND";
        landMsg.duration = 3.0f;
        enqueue_command(landMsg);
    }
}

void rigidbody::log(logger::log_type msgType, std::string message) {
    logger::post_log(msgType, this->tag, logPublisher, message);
}

void rigidbody::log_coord(logger::log_type msgType, std::string dataLabel, geometry_msgs::Vector3 data) {
    std::string message = dataLabel;
    message += ": [";
    message += std::to_string(data.x) + ", " + std::to_string(data.y) + ", " + std::to_string(data.z) +"]";
    logger::post_log(msgType, this->tag, logPublisher, message);
}


void rigidbody::update_current_flight_state() {
    if (this->get_state() == flight_state::DELETED) {
        return;
    }

    /* determine whether the last few poses are significantly different to suggest the drone is moving */
    geometry_msgs::Vector3& vel = currentVelocity.linear;
    double velocityMag = sqrt((vel.x * vel.x) + (vel.y * vel.y) + (vel.z * vel.z));
    // @TODO: find a good value for this
    bool droneHasMoved = velocityMag > 0.02;

    /* determine whether the drone is considered to be on the ground */
    /* in this case, if the drone is less than 5cm in from its home height it is considered landed */
    // @TODO: find a good value for this
    bool droneIsOnTheGround = (motionCapture.back().pose.position.z < (homePosition.z + 0.05));


    /* switch rigidbody's state to the detected state */
    if (droneHasMoved && ros::Time::now().toNSec() <= commandEnd.toNSec()) {
        this->set_state(flight_state::MOVING);
    } else if (!droneIsOnTheGround) {
        /* if the drone is in flight and the drone has no velocity: 'HOVER' */
        if (this->get_state() == flight_state::MOVING) {
            // @TODO: FIX for landing (think of crazyflie landing routine, it will go into hover mode 5cm above ground)
            /* drone has flicked from moving to hover, i.e. it has completed a command
             * if the drone has any remaining commands begin that, otherwise go into timeout stage 1 */
            if (!commandQueue.empty()) {
                this->log(logger::DEBUG, "Performing next queued command");
                handle_command();
            } else {
                /* Timeout stage 1 */
                this->do_stage_1_timeout();
            }
        }
    } else {
        /* drone is not moving and it is on the ground */
        this->set_state(flight_state::LANDED);
    }

}

void rigidbody::set_state(const flight_state& inputState) {
    if (inputState != this->get_state()) {
        this->log(logger::INFO, "Setting state to " + get_flight_state_string(inputState));
        this->state = inputState;
        droneHandle.setParam("mdp/drone_" + std::to_string(this->numericID) + "/state", get_flight_state_string(this->state));
    }
}

const rigidbody::flight_state &rigidbody::get_state() const {
    return this->state;
}

std::string rigidbody::get_flight_state_string(rigidbody::flight_state input) {
    switch (input) {
        case UNKNOWN:           return "UNKNOWN";
        case LANDED:            return "LANDED";
        case HOVERING:          return "HOVERING";
        case MOVING:            return "MOVING";
        case DELETED:           return "DELETED";
    }
}

void rigidbody::declare_expected_state(rigidbody::flight_state inputState) {
    this->set_state(inputState);
    this->declaredStateEndTime = ros::Time::now().toSec() + 0.1;
}

void rigidbody::do_stage_1_timeout() {
    this->log(logger::DEBUG, "Timeout stage 1");
    this->hover(TIMEOUT_HOVER+1.0f);
    this->hoverTimer.reset_timer(TIMEOUT_HOVER, true);
    this->set_state(flight_state::HOVERING);
}

const std::string& rigidbody::get_tag() {
    return this->tag;
}

uint32_t rigidbody::get_id() {
    return this->numericID;
}


void mdp_timer::reset_timer(double duration, bool Stage1Timeout) {
    this->timeoutTime = ros::Time::now().toSec() + duration;
    this->timerIsActive = true;
    this->isStage1Timeout = Stage1Timeout;
}

bool mdp_timer::has_timed_out() {
    if (timerIsActive) {
        if (ros::Time::now().toSec() >= this->timeoutTime) {
            this->timerIsActive = false;
            return true;
        }
    }
    return false;
}

void mdp_timer::close_timer() {
    this->timerIsActive = false;
}

bool mdp_timer::is_stage_timeout() const {
    return this->isStage1Timeout;
}
