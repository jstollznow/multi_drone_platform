//
// Created by jacob on 29/4/20.
//

#include "../drone_server/element_conversions.cpp"
#include "static_physical_management.h"

#define NAIVE_ACCEL_BUFFER 1.1f
// @TODO: Use these two methods from utility functions rather than creating copies
template<class T>
T multiply_by_constant(T a, double multiple) {
    T ret;
    ret.x = a.x * multiple;
    ret.y = a.y * multiple;
    ret.z = a.z * multiple;
    return ret;
}
template<class T>
double magnitude(T a) {
    double sum = a.x * a.x;
    sum += a.y * a.y;
    sum += a.z * a.z;
    return std::sqrt(sum);
}

static_limits static_physical_management::staticBoundary(
    {{-3.00, 3.00}},
    {{-3.00, 3.00}},
    {{0.10, 3.00}}
);

geometry_msgs::Vector3 static_physical_management::predict_position(ros::Time lastUpdate, geometry_msgs::Twist currVel, geometry_msgs::Pose currPos, int timeSteps) {
    double timeSinceMoCapUpdate = ros::Time::now().toSec() - lastUpdate.toSec();
    if (timeSinceMoCapUpdate < 0.0) {
        ROS_WARN("time since update returning less than 0");
        timeSinceMoCapUpdate = 0.0;
    }

    geometry_msgs::Vector3 pos = mdp_conversions::point_to_vector3(currPos.position);
    pos.x += (currVel.linear.x * timeSinceMoCapUpdate * (double)timeSteps);
    pos.y += (currVel.linear.y * timeSinceMoCapUpdate * (double)timeSteps);
    pos.z += (currVel.linear.z * timeSinceMoCapUpdate * (double)timeSteps);

    return pos;
}

double static_physical_management::predict_current_yaw(ros::Time lastUpdate, geometry_msgs::Twist currVel, geometry_msgs::Pose currPos, int timeSteps) {
    double timeSinceMoCapUpdate = ros::Time::now().toSec() - lastUpdate.toSec();
    if (timeSinceMoCapUpdate < 0.0) {
        ROS_WARN("time since update returning less than 0");
        timeSinceMoCapUpdate = 0.0;
    }

    double yaw = mdp_conversions::get_yaw_from_pose(currPos);
    yaw += (currVel.angular.z * timeSinceMoCapUpdate * (double)timeSteps);

    return yaw;
}

std::array<double, 2> static_physical_management::individual_velocity_boundaries(std::array<double, 2> limit, double currPos, double accel) {
    std::array<double, 2> ret;
    for(int i = 0; i < 2; i++) {
        double dir = (currPos > limit[i]) ? -1.0 : 1.0;
        double mag = std::sqrt(2.0 * accel * std::abs(currPos - limit[i]));
        ret[i] = dir * mag;
    }
    return ret;
}
static_limits static_physical_management::generate_velocity_boundaries(geometry_msgs::Vector3 currPos, double accel) {
    static_limits velocity;
    velocity.x = individual_velocity_boundaries(staticBoundary.x, currPos.x, accel);
    velocity.y = individual_velocity_boundaries(staticBoundary.y, currPos.y, accel);
    velocity.z = individual_velocity_boundaries(staticBoundary.z, currPos.z, accel);
    return velocity;
}
geometry_msgs::Vector3 static_physical_management::vel_static_limits(rigidbody* d, geometry_msgs::Vector3 requestedVelocity) {
    // time steps, how far in advance should we predict position
    auto positionPrediction = predict_position(d->timeOfLastApiUpdate, d->currentVelocity, d->currentPose, 0);

    // acceleration, how quickly can we slow down
    // can change this to have different for x,y,z
    auto velocityLimits = generate_velocity_boundaries(positionPrediction, 2);

    geometry_msgs::Vector3 limitAdjustedVel;

    limitAdjustedVel.x = std::min(std::max(requestedVelocity.x, velocityLimits.x[0]), velocityLimits.x[1]);
    limitAdjustedVel.y = std::min(std::max(requestedVelocity.y, velocityLimits.y[0]), velocityLimits.y[1]);
    limitAdjustedVel.z = std::min(std::max(requestedVelocity.z, velocityLimits.z[0]), velocityLimits.z[1]);
    return limitAdjustedVel;
}

geometry_msgs::Point static_physical_management::pos_static_limits(rigidbody *d, geometry_msgs::Point requestedPosition, double dur) {
    geometry_msgs::Vector3 velocity;
    velocity.x = (requestedPosition.x) / dur;
    velocity.y = (requestedPosition.y) / dur;
    velocity.z = (requestedPosition.z) / dur;

    d->log_coord<geometry_msgs::Point>(logger::DEBUG, "requestedPos", requestedPosition);
    d->log_coord<geometry_msgs::Vector3>(logger::DEBUG, "Inferred velocity", velocity);

    auto velLimits = vel_static_limits(d, velocity);
    geometry_msgs::Point limitAdjustedPos;
    limitAdjustedPos.x = velLimits.x * dur;
    limitAdjustedPos.y = velLimits.y * dur;
    limitAdjustedPos.z = velLimits.z * dur;

    return limitAdjustedPos;
}

geometry_msgs::Vector3 static_physical_management::check_physical_limits(geometry_msgs::Vector3 requestedPosition) {
    geometry_msgs::Vector3 limited;
    limited.x = std::min(std::max(staticBoundary.x[0], requestedPosition.x), staticBoundary.x[1]);
    limited.y = std::min(std::max(staticBoundary.y[0], requestedPosition.y), staticBoundary.y[1]);
    limited.z = std::min(std::max(staticBoundary.z[0], requestedPosition.z), staticBoundary.z[1]);
    return limited;
}

geometry_msgs::Vector3 static_physical_management::check_physical_limits(rigidbody* d, geometry_msgs::Vector3 requestedVelocity) {
    geometry_msgs::Vector3 limited;
    limited.x = std::min(std::max(requestedVelocity.x, d->velocity_limits.x[0]), d->velocity_limits.x[1]);
    limited.y = std::min(std::max(requestedVelocity.y, d->velocity_limits.y[0]), d->velocity_limits.y[1]);
    limited.z = std::min(std::max(requestedVelocity.z, d->velocity_limits.z[0]), d->velocity_limits.z[1]);
    return limited;
}

geometry_msgs::Vector3 static_physical_management::adjust_for_physical_limits(rigidbody* d, geometry_msgs::Vector3 requestedVelocity) {
    return check_physical_limits(d, requestedVelocity);
}

double static_physical_management::adjust_for_physical_limits(rigidbody* d, geometry_msgs::Vector3& requestedPosition, double dur) {
    auto pos_within_bounds = check_physical_limits(requestedPosition);
    geometry_msgs::Vector3 velocity;
    geometry_msgs::Vector3 distToTravel;
    distToTravel.x = (pos_within_bounds.x - d->currentPose.position.x);
    distToTravel.y = (pos_within_bounds.y - d->currentPose.position.y);
    distToTravel.z = (pos_within_bounds.z - d->currentPose.position.z);

    velocity.x = distToTravel.x / dur;
    velocity.y = distToTravel.y / dur;
    velocity.z = distToTravel.z / dur;

    auto vel_limits = check_physical_limits(d, velocity);
    if (magnitude(vel_limits) > d->maxVel) {
        vel_limits = multiply_by_constant(vel_limits, d->maxVel/magnitude(vel_limits));
    }
    double requiredDuration = magnitude(distToTravel)/magnitude(vel_limits);
    requiredDuration *= NAIVE_ACCEL_BUFFER;
    d->log(logger::DEBUG, "Set duration " + std::to_string(dur) + " Required: " + std::to_string(requiredDuration));
    d->log_coord(logger::DEBUG, "Position within bounds", pos_within_bounds);
    requestedPosition.x = pos_within_bounds.x;
    requestedPosition.y = pos_within_bounds.y;
    requestedPosition.z = pos_within_bounds.z;
    if (std::isnan(requiredDuration)) return dur;
    return std::max(requiredDuration, dur);
}

void static_physical_management::make_absolute_position(rigidbody *d, multi_drone_platform::api_update &msg) {
    if (msg.relativeXY) {
        msg.posVel.x += d->currentPose.position.x;
        msg.posVel.y += d->currentPose.position.y;
        msg.relativeXY = false;
    }
    if (msg.relativeZ){
        msg.posVel.z += d->currentPose.position.z;
        msg.relativeZ = false;
    }
}

void static_physical_management::make_absolute_velocity(rigidbody *d, multi_drone_platform::api_update &msg) {
    if (msg.relativeXY) {
        msg.posVel.x += d->currentVelocity.linear.x;
        msg.posVel.y += d->currentVelocity.linear.y;
        msg.relativeXY = false;
    }
    if (msg.relativeZ){
        msg.posVel.z += d->currentVelocity.linear.z;
        msg.relativeZ = false;
    }
}

void static_physical_management::check_land(rigidbody *d, multi_drone_platform::api_update &msg) {
    double velZ = d->currentPose.position.z / msg.duration;
    velZ = std::min(d->velocity_limits.z[1], std::max(velZ, d->velocity_limits.z[0]));
    float reqDuration = (float)(d->currentPose.position.z / velZ) * NAIVE_ACCEL_BUFFER;
    if (!std::isnan(reqDuration)) {
        msg.duration = std::max(msg.duration, reqDuration);
    }
}

void static_physical_management::check_go_home(rigidbody* d, multi_drone_platform::api_update &msg) {
    geometry_msgs::Vector3 vel;
    geometry_msgs::Vector3 distToTravel;
    distToTravel.x = d->homePosition.x - d->currentPose.position.x;
    distToTravel.y = d->homePosition.y - d->currentPose.position.y;
    vel.y = d->homePosition.y / msg.duration;
    vel.y = std::min(d->velocity_limits.y[1], std::max(vel.y, d->velocity_limits.y[0]));

    vel.x = d->homePosition.x / msg.duration;
    vel.x = std::min(d->velocity_limits.x[1], std::max(vel.x, d->velocity_limits.x[0]));

    vel.z = 0.0f;

    if (magnitude(vel) > d->maxVel) {
        vel = multiply_by_constant(vel, d->maxVel/magnitude(vel));
    }

    float reqDuration = (float)(magnitude(distToTravel)/magnitude(vel)) * NAIVE_ACCEL_BUFFER;

    if (!std::isnan(reqDuration)) {
        msg.duration = std::max(reqDuration, msg.duration);
    }

}

multi_drone_platform::api_update static_physical_management::adjust_command(rigidbody* d, const multi_drone_platform::api_update msg) {
    auto modifiedMsg = msg;
    if (d->maxVel == -1.0) d->set_max_vel();
//    if duration is less than or equal to 0, opt for default duration
    if (modifiedMsg.duration <= 0.0) modifiedMsg.duration = 4.0f;
    switch(apiMap[modifiedMsg.msgType]) {
        /* VELOCITY */
        case 0:
            make_absolute_velocity(d, modifiedMsg);
            modifiedMsg.posVel = adjust_for_physical_limits(d, modifiedMsg.posVel);
            break;
            /* POSITION */
        case 1:
            make_absolute_position(d, modifiedMsg);
            modifiedMsg.duration = adjust_for_physical_limits(d, modifiedMsg.posVel, modifiedMsg.duration);
            break;
            /* TAKEOFF */
        case 2:
            // already in absolute form, no need to convert.
            // simply check enough time has been allowed, and the height is not out of bounds
            if (modifiedMsg.posVel.z <= 0.0f) modifiedMsg.posVel.z = 0.25f;
            modifiedMsg.duration = adjust_for_physical_limits(d, modifiedMsg.posVel, modifiedMsg.duration);
            break;
            /* LAND */
        case 3:
            // checks enough duration has been added.
            check_land(d, modifiedMsg);
            break;
            /* HOVER */
        case 4:
            break;
            /* EMERGENCY */
        case 5:

            break;
            /* SET_HOME */
        case 6:
            make_absolute_position(d, modifiedMsg);
            break;
            /* GOTO_HOME */
        case 8:
            check_go_home(d, modifiedMsg);
            // don't need to check land as 4.0f is allowed on rigidbody which is plenty of time
            break;
        default:
            d->log(logger::WARN, "The API command, " + modifiedMsg.msgType + ", is not valid");
            break;
    }
    return modifiedMsg;
}
