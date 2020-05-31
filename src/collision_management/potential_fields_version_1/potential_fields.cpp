//
// Created by jacob on 18/5/20.
//
#include "potential_fields.h"
#include "utility_functions.cpp"
#define MIN_DIST 0.50f
#define GRAV_GAIN 0.25f
#define REPLUSIVE_GAIN 8.00f
#define COORD_GAIN 10.0f
#define FORWARD_STEPS 10

double closest = 1000.0f;
double closestThisRound = 1000.0f;
double lastClosestRound = 1000.0f;
bool potential_fields::check(rigidbody* d, std::vector<rigidbody*>& rigidbodies) {
    auto remainingDuration = d->commandEnd.toSec() - ros::Time().now().toSec();

    geometry_msgs::Vector3 velocity;
    geometry_msgs::Point posLimited;
    if (remainingDuration > 0.00) {
//        d->log(logger::INFO, "Remaining dur: " + std::to_string(remainingDuration));
//    @TODO: This is currently not configured for yaw
        switch(apiMap[d->lastRecievedApiUpdate.msgType]) {
            /* VELOCITY */
            case 0:
//                velLimited = vel_static_limits(d, d->desiredVelocity.linear);
//                if (!coord_equality(velLimited, d->desiredVelocity.linear)) {
//                    d->set_desired_velocity(velLimited, 0.0, remainingDuration, true, true);
//                }
                break;
                /* POSITION */
            case 1:
                position_based_pf(d, rigidbodies);

                break;
        }
    }
}
bool potential_fields::check_influence_mag(geometry_msgs::Vector3 replusiveForces) {
    return std::abs(replusiveForces.x) <= 0.1 &&
           std::abs(replusiveForces.y) <= 0.1 &&
           std::abs(replusiveForces.z) <= 0.1;
}

geometry_msgs::Vector3 predict_position(ros::Time lastUpdate, geometry_msgs::Twist currVel, geometry_msgs::Pose currPos, int timeSteps) {
    double timeSinceMoCapUpdate = ros::Time::now().toSec() - lastUpdate.toSec();
    if (timeSinceMoCapUpdate < 0.0) {
        ROS_WARN("time since update returning less than 0");
        timeSinceMoCapUpdate = 0.0;
    }

    geometry_msgs::Vector3 pos = utility_functions::point_to_vec3(currPos.position);
    pos.x += (currVel.linear.x * timeSinceMoCapUpdate * (double)timeSteps);
    pos.y += (currVel.linear.y * timeSinceMoCapUpdate * (double)timeSteps);
    pos.z += (currVel.linear.z * timeSinceMoCapUpdate * (double)timeSteps);

    return pos;
}

void potential_fields::position_based_pf(rigidbody *d, std::vector<rigidbody *> &rigidbodies) {
    geometry_msgs::Vector3 netForce;
    auto remainingDuration = d->commandEnd.toSec() - ros::Time().now().toSec();
    double t = 0.01;
    auto replusiveForces = replusive_forces(d, rigidbodies);
    netForce = utility_functions::add_vec3_or_point(replusiveForces, attractive_forces(d));
    geometry_msgs::Vector3 externalVelocityInfluence =
            utility_functions::multiply_by_constant(netForce, t / (d->mass));
//    geometry_msgs::Vector3 reqVel = calculate_req_velocity(d, remainingDuration);
//    externalVelocityInfluence = utility_functions::multiply_by_constant(externalVelocityInfluence, utility_functions::magnitude(reqVel) / utility_functions::magnitude(externalVelocityInfluence));
    geometry_msgs::Vector3 nextPos;
    nextPos.x = externalVelocityInfluence.x * remainingDuration;
    nextPos.y = externalVelocityInfluence.y * remainingDuration;
    nextPos.z = externalVelocityInfluence.z * remainingDuration;
    geometry_msgs::Vector3 nextVel;
    nextVel.x = d->currentVelocity.linear.x + externalVelocityInfluence.x;
    nextVel.y = d->currentVelocity.linear.y + externalVelocityInfluence.y;
    nextVel.z = d->currentVelocity.linear.z + externalVelocityInfluence.z;
    nextVel = utility_functions::multiply_by_constant(nextVel,
            utility_functions::magnitude(calculate_req_velocity(d, remainingDuration))/utility_functions::magnitude(nextVel));
    closest = std::min(closest, closestThisRound);

    d->log(logger::DEBUG, "Distance to closest " + std::to_string(closestThisRound));
    d->log_coord(logger::DEBUG, "Replusive Forces", replusiveForces);
    d->log_coord(logger::DEBUG, "External Velocity", externalVelocityInfluence);
    d->log_coord(logger::DEBUG, "NetForces", netForce);

    d->set_desired_velocity(nextVel, 0.0, remainingDuration);
    d->log(logger::INFO, "Closest dist: " + std::to_string(closest));
    lastClosestRound = closestThisRound;
    closestThisRound = 1000.0f;
}

geometry_msgs::Vector3 potential_fields::replusive_forces(rigidbody *d, std::vector<rigidbody *> &rigidbodies) {
    geometry_msgs::Vector3 replusiveForce;
    std::multimap<double, geometry_msgs::Pose> sortedObstacles;

    for (auto rb : rigidbodies) {
        if (rb->get_id() != d->get_id()) {
            double timeSteps =
                    utility_functions::magnitude(
                            utility_functions::difference(
                                    rb->currentVelocity.linear,
                                    rb->currentVelocity.linear))
                    * FORWARD_STEPS;
            auto obPoint = utility_functions::point_to_vec3(rb->currentPose.position);
            auto obFuturePoint = predict_position(rb->lastUpdate, rb->currentVelocity, rb->currentPose, timeSteps);
            auto dPoint = utility_functions::point_to_vec3(d->currentPose.position);
            double dist = utility_functions::distance_between(dPoint, obPoint);
            geometry_msgs::Pose obstacle;
            obstacle.orientation.w = rb->get_id();
            obstacle.orientation.x = dist;
            obstacle.position = utility_functions::vec3_to_point(utility_functions::difference(obPoint, dPoint));
            sortedObstacles.insert(std::pair<double, geometry_msgs::Pose>(dist, obstacle));
            closestThisRound = std::min(closestThisRound, utility_functions::distance_between(dPoint, obPoint));
            if (dist <= MIN_DIST) {
                double multiple = (REPLUSIVE_GAIN * std::pow((1.0 / dist) - (1.0 / MIN_DIST), 2.0)) / std::pow(dist, 3.0);
                auto diffVec = utility_functions::difference(dPoint, obPoint);
                replusiveForce.x += multiple * diffVec.x;
                replusiveForce.y += multiple * diffVec.y;
                replusiveForce.z += multiple * diffVec.z;
            }
        }
    }
    // iterate walls

    geometry_msgs::PoseArray msg;
    std_msgs::Float64 closestMsg;
    closestMsg.data = (float)closestThisRound;
    msg.header.stamp = ros::Time::now();
    for (auto ob : sortedObstacles) {
        msg.poses.push_back(ob.second);
    }
    d->obstaclesPublisher.publish(msg);
    d->closestObstaclePublisher.publish(closestMsg);
    return replusiveForce;
}
geometry_msgs::Vector3 potential_fields::coordination_force(rigidbody* d) {
    geometry_msgs::Vector3 coordForce;
    auto distToTarget = utility_functions::distance_between(utility_functions::point_to_vec3(d->currentPose.position), d->lastRecievedApiUpdate.posVel);
    if (distToTarget < 0.01) return coordForce;
    double multiple = COORD_GAIN * distToTarget * std::pow((1.0 / distToTarget) - (1.0 / MIN_DIST), 2.0);
    coordForce.x = coordForce.y = coordForce.z = multiple;
    return coordForce;
}

geometry_msgs::Vector3 potential_fields::attractive_forces(rigidbody *d) {
    geometry_msgs::Vector3 attractiveForce;
    auto diffVec = utility_functions::difference(utility_functions::point_to_vec3(d->currentPose.position), d->lastRecievedApiUpdate.posVel);
    double multiple = -GRAV_GAIN;
    attractiveForce.x = multiple * diffVec.x;
    attractiveForce.y = multiple * diffVec.y;
    attractiveForce.z = multiple * diffVec.z;
    return attractiveForce;
}

geometry_msgs::Vector3 potential_fields::calculate_req_velocity(rigidbody *d, double remainingDuration) {
    geometry_msgs::Vector3 reqVel;
    reqVel.x = (d->lastRecievedApiUpdate.posVel.x - d->currentPose.position.x) / remainingDuration;
    reqVel.y = (d->lastRecievedApiUpdate.posVel.y - d->currentPose.position.y) / remainingDuration;
    reqVel.z = (d->lastRecievedApiUpdate.posVel.z - d->currentPose.position.z) / remainingDuration;
    return reqVel;
}


