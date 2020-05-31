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
    static geometry_msgs::Vector3 attractive_forces(rigidbody* d);
    static geometry_msgs::Vector3 calculate_req_velocity(rigidbody* d, double remainingDuration);
    static bool check_influence_mag(geometry_msgs::Vector3 replusiveForces);
    static void position_based_pf(rigidbody* d, std::vector<rigidbody *> &rigidbodies);
public:
    static bool check(rigidbody* d, std::vector<rigidbody*>& rigidbodies);


    static geometry_msgs::Vector3 coordination_force(rigidbody *d);



};


#endif //MULTI_DRONE_PLATFORM_POTENTIAL_FIELDS_H
