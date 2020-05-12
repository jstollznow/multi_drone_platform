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
    velocity.y = individual_velocity_boundaries(staticBoundary.y, currPos.y, accel);
    velocity.z = individual_velocity_boundaries(staticBoundary.z, currPos.z, accel);
    return velocity;
}
geometry_msgs::Vector3 collision_management::vel_static_limits(rigidbody* d, geometry_msgs::Vector3 requestedVelocity) {
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

geometry_msgs::Point collision_management::pos_static_limits(rigidbody *d, geometry_msgs::Point requestedPosition, double dur) {
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

template <class T>
bool collision_management::coord_equality(T vec1, T vec2) {
    return (vec1.x == vec2.x) && (vec1.y == vec2.y) && (vec1.z == vec2.z);
}

geometry_msgs::Vector3 collision_management::check_physical_limits(geometry_msgs::Vector3 requestedPosition) {
    geometry_msgs::Vector3 limited;
    limited.x = std::min(std::max(staticBoundary.x[0], requestedPosition.x), staticBoundary.x[1]);
    limited.y = std::min(std::max(staticBoundary.y[0], requestedPosition.y), staticBoundary.y[1]);
    limited.z = std::min(std::max(staticBoundary.z[0], requestedPosition.z), staticBoundary.z[1]);
    return limited;
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

double collision_management::adjust_for_physical_limits(rigidbody* d, geometry_msgs::Vector3& requestedPosition, double dur) {
    auto pos_within_bounds = check_physical_limits(requestedPosition);
    geometry_msgs::Vector3 velocity;
    velocity.x = (pos_within_bounds.x) / dur;
    velocity.y = (pos_within_bounds.y) / dur;
    velocity.z = (pos_within_bounds.z) / dur;

    auto vel_limits = check_physical_limits(d, velocity);

    double requiredDuration = (vel_limits.x != 0) ? pos_within_bounds.x / vel_limits.x : 0.0;
    requiredDuration = std::max((vel_limits.y != 0) ? pos_within_bounds.y / vel_limits.y : 0.0, requiredDuration);
    requiredDuration = std::max((vel_limits.z != 0) ? pos_within_bounds.z / vel_limits.z : 0.0, requiredDuration);
    d->log(logger::DEBUG, "Set duration " + std::to_string(dur) + " Required: " + std::to_string(requiredDuration));
    d->log_coord(logger::DEBUG, "Position within bounds", pos_within_bounds);
    requestedPosition.x = pos_within_bounds.x;
    requestedPosition.y = pos_within_bounds.y;
    requestedPosition.z = pos_within_bounds.z;
    return std::max(requiredDuration, dur);
}

geometry_msgs::Vector3 collision_management::point_to_vec3(geometry_msgs::Point input) {
    geometry_msgs::Vector3 ret;
    ret.x = input.x;
    ret.y = input.y;
    ret.z = input.z;
    return ret;
}

bool collision_management::check(rigidbody* d, std::vector<rigidbody*>& rigidbodies) {
    double remainingDuration = d->commandEnd.toSec() - ros::Time().now().toSec();
    geometry_msgs::Vector3 velLimited;
    geometry_msgs::Point posLimited;
    if (remainingDuration > 0.00) {
//        d->log(logger::INFO, "Remaining dur: " + std::to_string(remainingDuration));
//    @TODO: This is currently not configured for yaw
        switch(apiMap[d->lastRecievedApiUpdate.msgType]) {
            /* VELOCITY */
            case 0:
                velLimited = vel_static_limits(d, d->desiredVelocity.linear);
                if (!coord_equality(velLimited, d->desiredVelocity.linear)) {
                    d->set_desired_velocity(velLimited, 0.0, remainingDuration, true, true);
                }
                break;
                /* POSITION */
            case 1:
//                this needs to be fixed, currently it is not producing the correct positions, need to rewrite
//                the pos_static_limits and need to allow des_position to allow a land command
//                needs to check velocity and adjust duration according to the position set

//                velLimited = vel_static_limits(d, d->currentVelocity.linear);
//                if (!coord_equality(velLimited, d->currentVelocity.linear)) {
//                    d->set_desired_velocity(velLimited, 0.0, remainingDuration, true, true);
//                }
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
}
