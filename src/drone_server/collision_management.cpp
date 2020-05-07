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
geometry_msgs::Vector3 collision_management::check_static_limits(rigidbody* d, geometry_msgs::Vector3 requestedVelocity) {
    // time steps, how far in advance should we predict position
    auto positionPrediction = predict_position(d->timeOfLastApiUpdate, d->currentVelocity, d->currentPose, 1);

    // acceleration, how quickly can we slow down
    auto velocityLimits = generate_velocity_boundaries(positionPrediction, 0.5);

    geometry_msgs::Vector3 limitAdjVel;

    limitAdjVel.x = std::min(std::max(requestedVelocity.x, velocityLimits.x[0]), velocityLimits.x[1]);
    limitAdjVel.y = std::min(std::max(requestedVelocity.y, velocityLimits.y[0]), velocityLimits.y[1]);
    limitAdjVel.z = std::min(std::max(requestedVelocity.z, velocityLimits.z[0]), velocityLimits.z[1]);
    d->log_coord(logger::DEBUG, "Velocity Limited", limitAdjVel);
    return limitAdjVel;
}
