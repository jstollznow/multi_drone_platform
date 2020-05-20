//
// Created by jacob on 18/5/20.
//

#include "potential_fields.h"

bool potential_fields::check(rigidbody* d, std::vector<rigidbody*>& rigidbodies) {
    double remainingDuration = d->commandEnd.toSec() - ros::Time().now().toSec();
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
                double t = 0.01;
                netForce = add_vec3_or_point(replusive_forces(d, rigidbodies), attractive_forces(d));
                geometry_msgs::Vector3 accelOverTime = multiply_by_constant(multiply_by_constant(netForce, 1/(d->mass)), t);
                geometry_msgs::Vector3 newVel;
                geometry_msgs::Vector3 reqVel = calculate_req_velocity(d, remainingDuration);
                newVel.x = reqVel.x + accelOverTime.x;
                newVel.y = reqVel.y + accelOverTime.y;
                newVel.z = reqVel.z + accelOverTime.z;

                d->log_coord(logger::DEBUG, "Net influence", netForce);
                d->log_coord(logger::DEBUG, "New Vel: ", newVel);
                d->set_desired_velocity(newVel, 0.0, remainingDuration, true, true);

                break;
        }
    }
}
template<class T>
T potential_fields::add_vec3_or_point(T a, T b) {
    T ret;
    ret.x = a.x + b.x;
    ret.y = a.y + b.y;
    ret.z = a.z + b.z;
    return ret;
}
geometry_msgs::Vector3 potential_fields::replusive_forces(rigidbody *d, std::vector<rigidbody *> &rigidbodies) {
    geometry_msgs::Vector3 replusiveForce;
    double scaling = 0.2;
    double minDist = 0.20;
    for (auto rb : rigidbodies) {
        if (rb->get_id() != d->get_id()) {
            geometry_msgs::Point obPoint = rb->currentPose.position;
            geometry_msgs::Point dPoint = d->currentPose.position;
            double dist = distance_between(dPoint, obPoint);

            if (dist <= minDist) {
                double multiple = (scaling * std::pow((1.0 / dist) - (1.0 / minDist), 2.0)) / std::pow(dist, 3.0);
                auto diffVec = difference(dPoint, obPoint);
//                d->log(logger::DEBUG, "multiple: " + std::to_string(multiple));
                replusiveForce.x = multiple * diffVec.x;
                replusiveForce.y = multiple * diffVec.y;
                replusiveForce.z = multiple * diffVec.z;

//                d->log_coord(logger::DEBUG, "Repulsive Forces from drone_" + std::to_string(rb->get_id()) + " @ dist: " + std::to_string(dist), replusiveForce);
            }

        }
    }
//    d->log_coord(logger::DEBUG, "Repulsive Forces Total", replusiveForce);
    return replusiveForce;
}

geometry_msgs::Vector3 potential_fields::attractive_forces(rigidbody *d) {
    geometry_msgs::Vector3 attractiveForce;
    double scaling = 1.0;
    auto diffVec = difference(d->currentPose.position, d->desiredPose.position);
    auto dist = distance_between(d->currentPose.position, d->desiredPose.position);
    if (dist < 0.01) {
        return attractiveForce;
    }
    double multiple = -scaling/dist;
    attractiveForce.x = multiple * diffVec.x;
    attractiveForce.y = multiple * diffVec.y;
    attractiveForce.z = multiple * diffVec.z;
//    d->log_coord(logger::DEBUG, "Attractive Force @ dist: " + std::to_string(dist), attractiveForce);
    return attractiveForce;
}

double potential_fields::distance_between(geometry_msgs::Point a, geometry_msgs::Point b) {
    double dist = 0.0f;
    dist += (a.x - b.x) * (a.x - b.x);
    dist += (a.y - b.y) * (a.y - b.y);
    dist += (a.z - b.z) * (a.z - b.z);
    return std::sqrt(dist);
}

template<class T>
T potential_fields::difference(T a, T b) {
    T ret;
    ret.x = a.x - b.x;
    ret.y = a.y - b.y;
    ret.z = a.z - b.z;
    return ret;
}

template<class T>
T potential_fields::multiply_by_constant(T a, double multiple) {
    T ret;
    ret.x = a.x * multiple;
    ret.y = a.y * multiple;
    ret.z = a.z * multiple;
    return ret;
}

geometry_msgs::Vector3 potential_fields::calculate_req_velocity(rigidbody *d, double remainingDuration) {
    geometry_msgs::Vector3 reqVel;
    reqVel.x = (d->desiredPose.position.x - d->currentPose.position.x) / remainingDuration;
    reqVel.y = (d->desiredPose.position.y - d->currentPose.position.y) / remainingDuration;
    reqVel.z = (d->desiredPose.position.z - d->currentPose.position.z) / remainingDuration;

    return reqVel;
}
