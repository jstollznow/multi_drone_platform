//
// Created by jacob on 18/5/20.
//

#include "potential_fields.h"
#include "utility_functions.cpp"

#define MIN_DIST 0.2f
#define GRAV_GAIN 1.0f
#define REPLUSIVE_GAIN 1.0f
#define COORD_GAIN 1.0f

bool potential_fields::check(rigidbody* d, std::vector<rigidbody*>& rigidbodies) {
    auto remainingDuration = d->commandEnd.toSec() - ros::Time().now().toSec();
    geometry_msgs::Vector3 netForce;
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
//                this needs to be fixed, currently it is not producing the correct positions, need to rewrite
//                the pos_static_limits and need to allow des_position to allow a land command
//                needs to check velocity and adjust duration according to the position set

//                velLimited = vel_static_limits(d, d->currentVelocity.linear);
//                if (!coord_equality(velLimited, d->currentVelocity.linear)) {
//                    d->set_desired_velocity(velLimited, 0.0, remainingDuration, true, true);
//                }
                maxVel.x = std::max(maxVel.x, d->currentVelocity.linear.x);
                maxVel.y = std::max(maxVel.y, d->currentVelocity.linear.y);
                maxVel.z = std::max(maxVel.z, d->currentVelocity.linear.z);

                double t = 0.01;
                netForce = utility_functions::add_vec3_or_point(replusive_forces(d, rigidbodies), attractive_forces(d));
//                netForce = add_vec3_or_point(netForce, coordination_force(d));
                geometry_msgs::Vector3 accelOverTime = utility_functions::multiply_by_constant(
                        utility_functions::multiply_by_constant(netForce, 1/(d->mass)),
                        t);
                geometry_msgs::Vector3 newVel;
                geometry_msgs::Vector3 reqVel = calculate_req_velocity(d, remainingDuration);
                newVel.x = reqVel.x + accelOverTime.x;
                newVel.y = reqVel.y + accelOverTime.y;
                newVel.z = reqVel.z + accelOverTime.z;

                d->log_coord(logger::DEBUG, "Max Vel", maxVel);
                d->log_coord(logger::DEBUG, "New Vel", newVel);
                d->set_desired_velocity(newVel, 0.0, remainingDuration, true, true);
                break;
        }
    }
}

geometry_msgs::Vector3 potential_fields::replusive_forces(rigidbody *d, std::vector<rigidbody *> &rigidbodies) {
    geometry_msgs::Vector3 replusiveForce;
    for (auto rb : rigidbodies) {
        if (rb->get_id() != d->get_id()) {
            geometry_msgs::Point obPoint = rb->currentPose.position;
            geometry_msgs::Point dPoint = d->currentPose.position;
            double dist = utility_functions::distance_between(dPoint, obPoint);

            if (dist <= MIN_DIST) {
                double multiple = (REPLUSIVE_GAIN * std::pow((1.0 / dist) - (1.0 / MIN_DIST), 2.0)) / std::pow(dist, 3.0);
                double L = std::pow(utility_functions::distance_between(d->currentPose.position, d->desiredPose.position), 2.0);
                auto diffVec = utility_functions::difference(dPoint, obPoint);
//                multiple *= L;
//                d->log(logger::DEBUG, "multiple: " + std::to_string(multiple));
                replusiveForce.x = multiple * diffVec.x;
                replusiveForce.y = multiple * diffVec.y;
                replusiveForce.z = multiple * diffVec.z;
            }

        }
    }
    return replusiveForce;
}
geometry_msgs::Vector3 potential_fields::coordination_force(rigidbody* d) {
    geometry_msgs::Vector3 coordForce;
    auto distToTarget = utility_functions::distance_between(d->currentPose.position, d->desiredPose.position);
    if (distToTarget < 0.01) return coordForce;
    double multiple = COORD_GAIN * distToTarget * std::pow((1.0 / distToTarget) - (1.0 / MIN_DIST), 2.0);
    coordForce.x = coordForce.y = coordForce.z = multiple;
    return coordForce;
}

geometry_msgs::Vector3 potential_fields::attractive_forces(rigidbody *d) {
    geometry_msgs::Vector3 attractiveForce;
    auto diffVec = utility_functions::difference(d->currentPose.position, d->desiredPose.position);
    auto distToTarget = utility_functions::distance_between(d->currentPose.position, d->desiredPose.position);
    if (distToTarget < 0.01) return attractiveForce;
    double multiple = -GRAV_GAIN / distToTarget;
    attractiveForce.x = multiple * diffVec.x;
    attractiveForce.y = multiple * diffVec.y;
    attractiveForce.z = multiple * diffVec.z;
    return attractiveForce;
}

geometry_msgs::Vector3 potential_fields::calculate_req_velocity(rigidbody *d, double remainingDuration) {
    geometry_msgs::Vector3 reqVel;
    reqVel.x = (d->desiredPose.position.x - d->currentPose.position.x) / remainingDuration;
    reqVel.y = (d->desiredPose.position.y - d->currentPose.position.y) / remainingDuration;
    reqVel.z = (d->desiredPose.position.z - d->currentPose.position.z) / remainingDuration;
    return reqVel;
}
