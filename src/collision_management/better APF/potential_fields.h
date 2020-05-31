//
// Created by jacob on 18/5/20.
//

#ifndef MULTI_DRONE_PLATFORM_POTENTIAL_FIELDS_H
#define MULTI_DRONE_PLATFORM_POTENTIAL_FIELDS_H

#include "rigidbody.h"

static geometry_msgs::Vector3 maxVel;

class potential_fields {
private:
    static geometry_msgs::Vector3 replusive_forces(rigidbody* d, std::vector<rigidbody*>& rigidbodies);
    static geometry_msgs::Vector3 attractive_forces(rigidbody* d, double remainingDuration);
    static geometry_msgs::Vector3 calculate_req_velocity(rigidbody* d, double remainingDuration);
    static geometry_msgs::Vector3 escape_local_minima(double speed);
    static void position_based_pf(rigidbody* d, std::vector<rigidbody *> &rigidbodies);
public:
    static bool check(rigidbody* d, std::vector<rigidbody*>& rigidbodies);


    static geometry_msgs::Vector3 tangential_force(rigidbody *d, geometry_msgs::Vector3 repulsive, geometry_msgs::Vector3 attractive);



};


#endif //MULTI_DRONE_PLATFORM_POTENTIAL_FIELDS_H
