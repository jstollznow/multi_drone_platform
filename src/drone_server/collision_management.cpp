//
// Created by jacob on 29/4/20.
//
#include "element_conversions.cpp"
#include "collision_management.h"

static_limits collision_management::staticBoundary(
    {{-3.00, 3.00}},
    {{-3.00, 3.00}},
    {{0.10, 3.00}}
);

geometry_msgs::Vector3 collision_management::predict_position(ros::Time lastUpdate, geometry_msgs::Twist currVel, geometry_msgs::Pose currPos, int timeSteps) {
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

double collision_management::predict_current_yaw(ros::Time lastUpdate, geometry_msgs::Twist currVel, geometry_msgs::Pose currPos, int timeSteps) {
    double timeSinceMoCapUpdate = ros::Time::now().toSec() - lastUpdate.toSec();
    if (timeSinceMoCapUpdate < 0.0) {
        ROS_WARN("time since update returning less than 0");
        timeSinceMoCapUpdate = 0.0;
    }

    double yaw = mdp_conversions::get_yaw_from_pose(currPos);
    yaw += (currVel.angular.z * timeSinceMoCapUpdate * (double)timeSteps);

    return yaw;
}

std::array<double, 2> collision_management::individual_velocity_boundaries(std::array<double, 2> limit, double currPos, double accel) {
    std::array<double, 2> ret;
    for(int i = 0; i < 2; i++) {
        double dir = (currPos > limit[i]) ? -1.0 : 1.0;
        double mag = std::sqrt(2.0 * accel * std::abs(currPos - limit[i]));
        ret[i] = dir * mag;
    }
    return ret;
}
static_limits collision_management::generate_velocity_boundaries(geometry_msgs::Vector3 currPos, double accel) {
    static_limits velocity;
    velocity.x = individual_velocity_boundaries(staticBoundary.x, currPos.x, accel);
    ROS_INFO("lower %.3f upper %.3f", velocity.x[0], velocity.x[1]);
    velocity.y = individual_velocity_boundaries(staticBoundary.y, currPos.y, accel);
    ROS_INFO("lower %.3f upper %.3f", velocity.y[0], velocity.y[1]);
    velocity.z = individual_velocity_boundaries(staticBoundary.z, currPos.z, accel);
    ROS_INFO("lower %.3f upper %.3f", velocity.z[0], velocity.z[1]);

    return velocity;
}
geometry_msgs::Vector3 collision_management::vel_static_limits(rigidbody* d, geometry_msgs::Vector3 requestedVelocity) {
    // time steps, how far in advance should we predict position
    auto positionPrediction = predict_position(d->timeOfLastApiUpdate, d->currentVelocity, d->currentPose, 1);

    // acceleration, how quickly can we slow down
    auto velocityLimits = generate_velocity_boundaries(positionPrediction, 0.5);

    geometry_msgs::Vector3 limitAdjustedVel;

    limitAdjustedVel.x = std::min(std::max(requestedVelocity.x, velocityLimits.x[0]), velocityLimits.x[1]);
    limitAdjustedVel.y = std::min(std::max(requestedVelocity.y, velocityLimits.y[0]), velocityLimits.y[1]);
    limitAdjustedVel.z = std::min(std::max(requestedVelocity.z, velocityLimits.z[0]), velocityLimits.z[1]);
    return limitAdjustedVel;
}

geometry_msgs::Vector3 collision_management::pos_static_limits(rigidbody *d, geometry_msgs::Vector3 requestedPosition, double dur) {
    geometry_msgs::Vector3 velocity;
    velocity.x = (requestedPosition.x) / dur;
    velocity.y = (requestedPosition.y) / dur;
    velocity.z = (requestedPosition.z) / dur;

    d->log_coord(logger::DEBUG, "requestedPos", requestedPosition);
    d->log_coord(logger::DEBUG, "Inferred velocity", velocity);

    auto velLimits = vel_static_limits(d, velocity);
    geometry_msgs::Vector3 limitAdjustedPos;
    limitAdjustedPos.x = velLimits.x * dur;
    limitAdjustedPos.y = velLimits.y * dur;
    limitAdjustedPos.z = velLimits.z * dur;

    return limitAdjustedPos;
}
bool collision_management::vector3_equality(geometry_msgs::Vector3 vec1, geometry_msgs::Vector3 vec2) {
    return (vec1.x == vec2.x) && (vec1.y == vec2.y) && (vec1.z == vec2.z);
}
geometry_msgs::Vector3 collision_management::check_physical_limits(rigidbody* d, geometry_msgs::Vector3 requestedVelocity) {
    geometry_msgs::Vector3 limited;
    limited.x = std::min(std::max(requestedVelocity.x, d->physical_limits.x[0]), d->physical_limits.x[1]);
    limited.y = std::min(std::max(requestedVelocity.y, d->physical_limits.y[0]), d->physical_limits.y[1]);
    limited.z = std::min(std::max(requestedVelocity.z, d->physical_limits.z[0]), d->physical_limits.z[1]);
    return limited;
}
geometry_msgs::Vector3 collision_management::adjust_for_physical_limits(rigidbody* d, geometry_msgs::Vector3 requestedVelocity) {
    return check_physical_limits(d, requestedVelocity);
}
double collision_management::adjust_for_physical_limits(rigidbody* d, geometry_msgs::Vector3 requestedPosition, double dur) {
    geometry_msgs::Vector3 velocity;
    velocity.x = (requestedPosition.x) / dur;
    velocity.y = (requestedPosition.y) / dur;
    velocity.z = (requestedPosition.z) / dur;
    auto velLimits = check_physical_limits(d, velocity);
    double requiredDuration;
    requiredDuration = (velocity.x != 0) ? requestedPosition.x / velocity.x : 0.0;
    requiredDuration = std::max((velocity.y != 0) ? requestedPosition.y / velocity.y : 0.0, requiredDuration);
    requiredDuration = std::max((velocity.z != 0) ? requestedPosition.z / velocity.z : 0.0, requiredDuration);
    return std::max(requiredDuration, dur);
}

bool collision_management::check(rigidbody* d, std::vector<rigidbody*>& rigidbodies) {
    double remainingDuration = d->commandEnd.toSec() - ros::Time().now().toSec();
    geometry_msgs::Vector3 limitedCommand;
//    @TODO: This is currently not configured for yaw
    switch(apiMap[d->lastRecievedApiUpdate.msgType]) {
        /* VELOCITY */
        case 0:
            limitedCommand = vel_static_limits(d, d->lastRecievedApiUpdate.posVel);
            if (!vector3_equality(limitedCommand, d->lastRecievedApiUpdate.posVel)) {
                d->set_desired_velocity(limitedCommand, 0.0, remainingDuration, true, true);
            }
            break;
            /* POSITION */
        case 1:
            limitedCommand = pos_static_limits(d, d->lastRecievedApiUpdate.posVel, remainingDuration);
            if (!vector3_equality(limitedCommand, d->lastRecievedApiUpdate.posVel)) {
                d->set_desired_position(limitedCommand, 0.0, remainingDuration, true, true);
            }
            break;
            /* TAKEOFF */
        case 2:

            break;
            /* LAND */
        case 3:

            break;
            /* HOVER */
        case 4:

            break;
            /* EMERGENCY */
        case 5:

            break;
            /* SET_HOME */
        case 6:

            break;
            /* GOTO_HOME */
        case 8:

            break;
        default:

            break;
    }
}
