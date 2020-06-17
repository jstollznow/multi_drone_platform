//
// Created by jacob on 18/5/20.
//
#include "potential_fields.h"
#include "utility_functions.cpp"

#define ATTRACTIVE_DIST 0.10f
#define K_P 6.0f
#define K_D 0.8f

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
geometry_msgs::Vector3 potential_fields::escape_local_minima(double speed) {
    geometry_msgs::Vector3 randomVec;
    randomVec.x = (2 * (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5));
    randomVec.y = (2 * (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5));
    randomVec.z = (2 * (static_cast <float> (rand()) / static_cast <float> (RAND_MAX) - 0.5));
    randomVec = utility_functions::multiply_by_constant(randomVec, speed / utility_functions::magnitude(randomVec));
    return randomVec;
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
    geometry_msgs::Vector3 netPotentialVelocity;

    auto remainingDuration = d->commandEnd.toSec() - ros::Time().now().toSec();
    auto repulsiveForces = replusive_forces(d, rigidbodies);
    auto attractiveForces = attractive_forces(d, remainingDuration);
    netPotentialVelocity = utility_functions::add_vec3_or_point(repulsiveForces, attractiveForces);
    closest = std::min(closest, closestThisRound);

    d->log(logger::DEBUG, "Distance to closest " + std::to_string(closestThisRound));
    d->log_coord(logger::DEBUG, "Repulsive Force", repulsiveForces);
    d->log_coord(logger::DEBUG, "NetForces", netPotentialVelocity);
    if (utility_functions::magnitude(netPotentialVelocity) <= 0.20) {
        d->log(logger::DEBUG, "Local Minima");
//        var halfWayVector = (Vector3.up + Vector3.right).normalized;
        auto tangVec = utility_functions::get_tangent_vec(attractiveForces, repulsiveForces);

//        netPotentialVelocity = utility_functions::multiply_by_constant(tangVec, 2);
    }

    if (utility_functions::magnitude(repulsiveForces) <= 0.2) {
        d->set_desired_position(d->lastRecievedApiUpdate.posVel, 0.0, remainingDuration);
    }
    else {
        d->set_desired_velocity(netPotentialVelocity, 0.0, remainingDuration);
    }
    d->log(logger::INFO, "Closest dist: " + std::to_string(closest));
    lastClosestRound = closestThisRound;
    closestThisRound = 1000.0f;
}

geometry_msgs::Vector3 potential_fields::replusive_forces(rigidbody *d, std::vector<rigidbody *> &rigidbodies) {
    geometry_msgs::Vector3 replusiveForce;
    std::multimap<double, geometry_msgs::Pose> sortedObstacles;
    for (auto rb : rigidbodies) {
        if (rb->get_id() != d->get_id()) {
            auto obPoint = utility_functions::point_to_vec3(rb->currentPose.position);
            auto dPoint = utility_functions::point_to_vec3(d->currentPose.position);
            auto dFuturePoint = predict_position(d->lastUpdate, d->currentVelocity, d->currentPose, 10);
            auto diffVec = utility_functions::difference(dFuturePoint, obPoint);
            double d0 = utility_functions::magnitude(diffVec);
            auto unitDirection = utility_functions::multiply_by_constant(diffVec, 1 / d0);

            double velDiff = utility_functions::distance_between(d->currentVelocity.linear, rb->currentVelocity.linear);

            geometry_msgs::Pose obstacle;
            obstacle.orientation.w = rb->get_id();
            obstacle.orientation.x = d0;
            obstacle.position = utility_functions::vec3_to_point(utility_functions::difference(obPoint, dPoint));
            sortedObstacles.insert(std::pair<double, geometry_msgs::Pose>(d0, obstacle));

            closestThisRound = std::min(closestThisRound, d0);

            d->log(logger::DEBUG, "Dist: " + std::to_string(d0));

            if (d0 <= rb->restrictedDistance) {
                replusiveForce.x += d->maxVel * unitDirection.x;
                replusiveForce.y += d->maxVel * unitDirection.y;
                replusiveForce.z += d->maxVel * unitDirection.z;
            }else if (d0 <= rb->influenceDistance) {
                double vr = K_P * (rb->influenceDistance - d0) + K_D * (velDiff);
                replusiveForce.x += vr * unitDirection.x;
                replusiveForce.y += vr * unitDirection.y;
                replusiveForce.z += vr * unitDirection.z;
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
geometry_msgs::Vector3 potential_fields::tangential_force(rigidbody* d, geometry_msgs::Vector3 repulsive, geometry_msgs::Vector3 attractive) {
    geometry_msgs::Vector3 coordForce;
    auto normalVec = utility_functions::get_tangent_vec(attractive, repulsive);
    normalVec = utility_functions::multiply_by_constant(normalVec, 2);
    return normalVec;
}

geometry_msgs::Vector3 potential_fields::attractive_forces(rigidbody *d, double remainingDuration) {
    geometry_msgs::Vector3 attractiveForce;
    auto distToTarget = utility_functions::distance_between(utility_functions::point_to_vec3(d->currentPose.position), d->lastRecievedApiUpdate.posVel);
    auto reqVelocity = calculate_req_velocity(d, remainingDuration);

    if (distToTarget <= ATTRACTIVE_DIST) {
        double multiple = utility_functions::magnitude(reqVelocity)/ATTRACTIVE_DIST;
        auto posDiff = utility_functions::difference(d->lastRecievedApiUpdate.posVel,utility_functions::point_to_vec3(d->currentPose.position));
        attractiveForce = utility_functions::multiply_by_constant(posDiff, multiple);
    }
    else {
        attractiveForce = reqVelocity;
    }

    return attractiveForce;
}

geometry_msgs::Vector3 potential_fields::calculate_req_velocity(rigidbody *d, double remainingDuration) {
    geometry_msgs::Vector3 reqVel;
    reqVel.x = (d->lastRecievedApiUpdate.posVel.x - d->currentPose.position.x) / remainingDuration;
    reqVel.y = (d->lastRecievedApiUpdate.posVel.y - d->currentPose.position.y) / remainingDuration;
    reqVel.z = (d->lastRecievedApiUpdate.posVel.z - d->currentPose.position.z) / remainingDuration;
    return reqVel;
}


