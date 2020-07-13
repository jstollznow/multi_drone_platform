#include <queue>
#include <std_msgs/Float32.h>
#include "rigidbody.h"
#include "element_conversions.cpp"
#include "../collision_management/static_physical_management.h"
#include "../collision_management/potential_fields.h"

rigidbody::rigidbody(std::string tag, uint32_t id): mySpin(1,&myQueue), icpObject(tag, droneHandle) {
    this->tag = tag;
    this->numericID = id;
    this->batteryDying = false;
    this->lastRecievedApiUpdate.msgType = "";

//    initialise physical velocity limits in m/s
    this->velocity_limits.x = {{-10.0, 10.0}};
    this->velocity_limits.y = {{-10.0, 10.0}};
    this->velocity_limits.z = {{-10.0, 10.0}};
    this->maxVel = -1.0;
    this->width = 0.0;
    this->length = 0.0;
    this->height = 0.0;
    this->restrictedDistance = 0.0;
    this->influenceDistance = 0.0;

    // assume rigidbody is declared with drone facing in positive x direction
    this->absoluteYaw = 0.0f;

    droneHandle.setParam("mdp/drone_" + std::to_string(this->get_id()) + "/width", this->width);
    droneHandle.setParam("mdp/drone_" + std::to_string(this->get_id()) + "/height", this->height);
    droneHandle.setParam("mdp/drone_" + std::to_string(this->get_id()) + "/length", this->length);
    droneHandle.setParam("mdp/drone_" + std::to_string(this->get_id()) + "/restrictedDistance", this->restrictedDistance);
    droneHandle.setParam("mdp/drone_" + std::to_string(this->get_id()) + "/influenceDistance", this->influenceDistance);

//    initialise approx. mass in kg
    this->mass = 0.100;

    // look for drone under tag namespace then vrpn output
    std::string idStr = std::to_string(numericID);

#if USE_NATNET
    std::string motionTopic = "/mocap/rigid_bodies/" + tag + "/pose";
#else /* USE VRPN */
    std::string motionTopic = "/vrpn_client_node/" + tag + "/pose";
#endif
    std::string logTopic = "mdp/drone_" + idStr + "/log";
    std::string apiTopic = "mdp/drone_" + idStr + "/apiUpdate";
    std::string batteryTopic = "mdp/drone_" + idStr + "/battery";

    std::string currPoseTopic = "mdp/drone_" + idStr + "/curr_pose";
    std::string desPoseTopic = "mdp/drone_" + idStr + "/des_pose";

    std::string currTwistTopic = "mdp/drone_" + idStr + "/curr_twist";
    std::string desTwistTopic = "mdp/drone_" + idStr + "/des_twist";

    std::string obstacleTopic = "mdp/drone_" + idStr +"/obstacles";
    std::string closestObstacleTopic = "mdp/drone_" + idStr + "/closest_obstacle";

    droneHandle = ros::NodeHandle();
    droneHandle.setCallbackQueue(&myQueue);

    apiPublisher = droneHandle.advertise<multi_drone_platform::api_update> (apiTopic, 2);
    apiSubscriber = droneHandle.subscribe(apiTopic, 2, &rigidbody::api_callback, this);
    logPublisher = droneHandle.advertise<multi_drone_platform::log> (logTopic, 20);
    motionSubscriber = droneHandle.subscribe<geometry_msgs::PoseStamped>(motionTopic, 1,&rigidbody::add_motion_capture, this);
    batteryPublisher = droneHandle.advertise<std_msgs::Float32>(batteryTopic, 1);

    currentPosePublisher = droneHandle.advertise<geometry_msgs::PoseStamped> (currPoseTopic, 1);
    desiredPosePublisher = droneHandle.advertise<geometry_msgs::PoseStamped>(desPoseTopic, 1);

    currentTwistPublisher = droneHandle.advertise<geometry_msgs::TwistStamped> (currTwistTopic, 1);
    desiredTwistPublisher = droneHandle.advertise<geometry_msgs::TwistStamped>(desTwistTopic, 1);

    obstaclesPublisher = droneHandle.advertise<geometry_msgs::PoseArray> (obstacleTopic, 1);
    closestObstaclePublisher = droneHandle.advertise<std_msgs::Float64> (closestObstacleTopic, 1);

    this->log(logger::INFO, "My id is: " + std::to_string(id));
    this->log(logger::INFO, "Subscribing to motion topic: " + motionTopic);
    this->log(logger::INFO, "Subscrbing to API topic: " + apiTopic);
    this->log(logger::INFO, "Publishing log data to: " + logTopic);
    this->log(logger::INFO, "Publishing current position to: " + currPoseTopic);
    this->log(logger::INFO, "Publishing desired position to: " + desPoseTopic);
    this->log(logger::INFO, "Publishing current velocity to: " + currTwistTopic);
    this->log(logger::INFO, "Publishing desired velocity to: " + desTwistTopic);
    this->log(logger::INFO, "Publishing obstacle array to: " + obstacleTopic);
    this->log(logger::INFO, "Publishing closest obstacle distance to: " + closestObstacleTopic);

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

void rigidbody::set_max_vel() {
    this->maxVel = std::max(std::abs(velocity_limits.x[0]), std::abs(velocity_limits.x[1]));
    this->maxVel = std::max(maxVel, std::abs(velocity_limits.y[0]));
    this->maxVel = std::max(maxVel, std::abs(velocity_limits.y[1]));
    this->maxVel = std::max(maxVel, std::abs(velocity_limits.z[0]));
    this->maxVel = std::max(maxVel, std::abs(velocity_limits.z[1]));
}

double rigidbody::vec3_distance(geometry_msgs::Vector3 a, geometry_msgs::Vector3 b)
{
    double dist = 0.0;
    dist += std::abs(a.x - b.x);
    dist += std::abs(a.y - b.y);
    dist += std::abs(a.z - b.z);
    return dist;
}

void rigidbody::set_desired_position(geometry_msgs::Vector3 pos, float yaw, float duration) {
    if (this->get_state() == flight_state::LANDED) {
        this->log(logger::WARN, "Go to position called on landed drone, ignoring");
        return;
    }
    // limits are processed in absolute form
    duration = static_physical_management::adjust_for_physical_limits(this, pos, duration);
    this->desiredPose.position.x = pos.x;
    this->desiredPose.position.y = pos.y;
    this->desiredPose.position.z = pos.z;
    // @TODO: need to manage orientation
    this->log_coord<geometry_msgs::Point>(logger::DEBUG, "Desired Position", this->desiredPose.position);
    this->log(logger::DEBUG, "Adjusted Dur: " + std::to_string(duration));
    geometry_msgs::PoseStamped desPoseMsg;
    desPoseMsg.pose = desiredPose;
    desPoseMsg.header.stamp = ros::Time::now();
    desiredPosePublisher.publish(desPoseMsg);

    /* declare flight state to moving if this is not a hover message */
    if (!(pos.x == 0.0f && pos.y == 0.0f && pos.z == 0.0f)) {
        this->declare_expected_state(flight_state::MOVING);
    }

    /* modify input yaw such that it takes the shortest route to the new yaw */
    yaw = std::fmod(yaw, 360.0f);

    float currentYaw = std::fmod(mdp_conversions::get_yaw_from_pose(this->get_current_pose()), 360.0f);

    float yawDiff = yaw - currentYaw;
    if (std::abs(yawDiff) > 180.0f) {
        yawDiff += 360.0f;
        yawDiff = std::fmod(yawDiff, 360.0f);
    }
    yaw = currentYaw + yawDiff;

    this->log(logger::WARN, "next yaw is: " + std::to_string(yaw));

    /* send to wrapper */
    this->on_set_position(pos, yaw, duration);
}

void rigidbody::set_desired_velocity(geometry_msgs::Vector3 vel, float yawRate, float duration) {
    if (this->get_state() == flight_state::LANDED) {
        this->log(logger::WARN, "set velocity called on landed drone, ignoring");
        return;
    }

    vel = static_physical_management::adjust_for_physical_limits(this, vel);
    this->log_coord<geometry_msgs::Vector3>(logger::DEBUG, "Vel Command", vel);
    this->log(logger::DEBUG, "Duration : " + std::to_string(duration));
    this->desiredVelocity.linear = vel;
    this->desiredVelocity.angular.z = yawRate;

    geometry_msgs::TwistStamped desTwistMsg;
    desTwistMsg.twist = desiredVelocity;
    desTwistMsg.header.stamp = ros::Time::now();

    desiredTwistPublisher.publish(desTwistMsg);

    this->declare_expected_state(flight_state::MOVING);
    this->on_set_velocity(vel, yawRate, duration);
}

bool rigidbody::is_msg_different(const multi_drone_platform::api_update& msg, const multi_drone_platform::api_update& last_message) const {
    double timeBetweenCommonMsgs = 0.5;
    double distanceBetweenCommonMsgs = 0.02;
    double timeBetweenTwoMsgs = ros::Time::now().toSec() - timeOfLastApiUpdate.toSec();
    if (timeBetweenTwoMsgs > timeBetweenCommonMsgs) return true; // there has been significant time between msgs
    if (msg.msgType != last_message.msgType) return true;        // they are not the same msg type
    if (msg.relativeXY != last_message.relativeXY) return true;    // they do not have the same xy relative
    if (msg.relativeZ != last_message.relativeZ) return true;      // they do not have the same z relative
    if (std::abs(msg.duration - (last_message.duration - timeBetweenTwoMsgs)) > 0.5) return true; // relative durations are significantly different
    if (vec3_distance(msg.posVel, last_message.posVel) > distanceBetweenCommonMsgs) return true; // they are significantly different in destinations
    if (std::abs(msg.yawVal - last_message.yawVal) > 10.0) return true; // they have significantly different yawvals
    return false;   // else they are pretty much the same message
}

geometry_msgs::Vector3 rigidbody::get_home_coordinates() {
    return homePosition;
}   

void rigidbody::set_home_coordiates(geometry_msgs::Vector3 pos) {
    // z coord does not matter for home position, will be set in each case
    homePosition.x = pos.x;
    homePosition.y = pos.y;
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

    // do stuff with coordinate systems
    motionMsg.header = msg->header;

    motionMsg.pose.position.x = msg->pose.position.y * -1;
    motionMsg.pose.position.y = msg->pose.position.x;
    motionMsg.pose.position.z = msg->pose.position.z;

    motionMsg.pose.orientation.x = msg->pose.orientation.y * -1;
    motionMsg.pose.orientation.y = msg->pose.orientation.x;
    motionMsg.pose.orientation.z = msg->pose.orientation.z;
    motionMsg.pose.orientation.w = msg->pose.orientation.w;

    motionMsg.header.frame_id = "mocap";


    if (motionCapture.empty()) {
        motionCapture.push(motionMsg);
        motionCapture.push(motionMsg);

        homePosition.x = motionMsg.pose.position.x;
        homePosition.y = motionMsg.pose.position.y;
        homePosition.z = motionMsg.pose.position.z;
        
        std::string homePosLog = "HOME POS: [" + std::to_string(homePosition.x) + ", " 
        + std::to_string(homePosition.y) + ", " + std::to_string(homePosition.z) + "]";

        this->absoluteYaw = mdp_conversions::get_yaw_from_pose(motionMsg.pose);

        this->log(logger::INFO, homePosLog);
    }

    /* otherwise, erase oldest message and push back newest message */
    motionCapture.pop();
    motionCapture.push(motionMsg);
    this->calculate_velocity();
    this->adjust_absolute_yaw();

    currentPose = motionCapture.back().pose;
    // ROS_INFO("Current Position: x: %f, y: %f, z: %f",currPos.position.x, currPos.position.y, currPos.position.z);
    // @TODO: Orientation implementation

    geometry_msgs::Vector3& vel = currentVelocity.linear;
    double velocityMag = sqrt((vel.x * vel.x) + (vel.y * vel.y) + (vel.z * vel.z));
//    this->log(logger::INFO, "vel = " + std::to_string(velocityMag) + "(z = " + std::to_string(vel.z) + ")");

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
    if (this->timeoutTimer.has_timed_out()) {
        if (this->timeoutTimer.is_stage_timeout()) {
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
    else if (this->get_state() == MOVING || this->get_state() == HOVERING){
        //potential_fields::check(this, rigidbodies);
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
            auto modMsg = static_physical_management::adjust_command(this, msg);
            this->commandQueue.push_back(modMsg);
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
    if (!commandQueue.empty()) {
        this->timeoutTimer.close_timer(); // stop timeout 2 from happening as drone has received a message
        bool isGoHomeMessage = false;
        multi_drone_platform::api_update msg = this->commandQueue.front();
        if (is_msg_different(msg, this->lastRecievedApiUpdate)) {
            this->log(logger::INFO, "=> Handling command: " + msg.msgType);
            this->log(logger::DEBUG, "Duration " + std::to_string(msg.duration));
            this->lastRecievedApiUpdate = msg;
            this->timeOfLastApiUpdate = ros::Time::now();
            this->commandEnd = timeOfLastApiUpdate + ros::Duration(msg.duration);
            switch(apiMap[msg.msgType]) {
                /* VELOCITY */
                case 0:
                    ROS_INFO("V: [%.2f, %.2f, %.2f] rel_Xy: %d, rel_z: %d, dur: %.1f", msg.posVel.x, msg.posVel.y, msg.posVel.z, msg.relativeXY, msg.relativeZ, msg.duration);
                    if (msg.relativeXY && msg.relativeZ) this->log(logger::ERROR, "This should have already been preprocessed");
                    set_desired_velocity(msg.posVel, msg.yawVal, msg.duration);
                    break;
                /* POSITION */
                case 1:
                    ROS_INFO("P: xyz: %.2f %.2f %.2f, rel_Xy: %d, rel_z: %d", msg.posVel.x, msg.posVel.y, msg.posVel.z, msg.relativeXY, msg.relativeZ);
                    if (msg.relativeXY && msg.relativeZ) this->log(logger::ERROR, "This should have already been preprocessed");
                    set_desired_position(msg.posVel, msg.yawVal, msg.duration);
                    break;
                /* TAKEOFF */
                case 2:
                    ROS_INFO("Height: %f", msg.posVel.z);
                    this->log(logger::INFO, "Duration: " + std::to_string(msg.duration));
                    takeoff(msg.posVel.z, msg.duration);
                    break;
                /* LAND */
                case 3:
                    land(msg.duration);
                    break;
                /* HOVER */
                case 4:
                    this->hover(msg.duration);
                    this->timeoutTimer.reset_timer(msg.duration - 0.05f);
                    break;
                /* EMERGENCY */
                case 5: 
                    emergency();
                    break;
                /* SET_HOME */
                case 6:
                    if (!msg.relativeXY) this->log(logger::ERROR, "This should have already been preprocessed");
                    set_home_coordiates(msg.posVel);
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
    if (!commandQueue.empty()) { // this is a hack fix
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
    this->log_coord<geometry_msgs::Point>(logger::DEBUG, "Position at hover request: ", currentPose.position);
    float yaw = mdp_conversions::get_yaw_from_pose(this->get_current_pose());
    set_desired_position(mdp_conversions::point_to_vector3(currentPose.position), yaw, duration);
    this->declare_expected_state(flight_state::MOVING, duration);
}

void rigidbody::go_home(float yaw, float duration, float in_height) {
    /* enqueue goto command */
    multi_drone_platform::api_update goMsg;
    goMsg.msgType = "POSITION";
    goMsg.duration = duration;
    goMsg.yawVal = yaw;
    goMsg.posVel = homePosition;
    goMsg.posVel.z = currentPose.position.z;
    enqueue_command(goMsg);

    /* enqueue land msg if necessary */
    if (in_height < 0.1f) {
        multi_drone_platform::api_update landMsg;
        landMsg.msgType = "LAND";
        landMsg.duration = 3.0f;
        enqueue_command(landMsg);
    }
}

void rigidbody::log(logger::log_type msgType, std::string message) {
    logger::post_log(msgType, this->tag, logPublisher, message);
}

template <class T>
void rigidbody::log_coord(logger::log_type msgType, std::string dataLabel, T data) {
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

void rigidbody::declare_expected_state(rigidbody::flight_state inputState, double duration) {
    this->set_state(inputState);
    this->declaredStateEndTime = ros::Time::now().toSec() + duration;
}

void rigidbody::do_stage_1_timeout() {
    this->log(logger::DEBUG, "Timeout stage 1");
    this->hover(TIMEOUT_HOVER);
    this->timeoutTimer.reset_timer(TIMEOUT_HOVER, true);
    this->set_state(flight_state::HOVERING);
}

const std::string& rigidbody::get_tag() {
    return this->tag;
}

uint32_t rigidbody::get_id() {
    return this->numericID;
}

const geometry_msgs::Pose& rigidbody::get_current_pose() const {
    return this->currentPose;
}

double rigidbody::get_end_yaw_from_yawrate_and_time_period(double yawrate, double time_period) const {
    return absoluteYaw + (yawrate * time_period);
}

void rigidbody::adjust_absolute_yaw() {
    auto newYaw = mdp_conversions::get_yaw_from_pose(motionCapture.front().pose);
    auto oldYaw = mdp_conversions::get_yaw_from_pose(motionCapture.back().pose);
    float yawDiff = newYaw - oldYaw;
    if (std::abs(yawDiff) > 180.0f) {
        yawDiff = std::fmod(360.0f + yawDiff, 360.f);
    }
    absoluteYaw += yawDiff;
}

ros::NodeHandle rigidbody::get_ros_node_handle() const {
    return this->droneHandle;
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
