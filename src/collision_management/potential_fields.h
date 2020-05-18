//
// Created by jacob on 18/5/20.
//

#ifndef MULTI_DRONE_PLATFORM_POTENTIAL_FIELDS_H
#define MULTI_DRONE_PLATFORM_POTENTIAL_FIELDS_H
#include "rigidbody.h"

class potential_fields {
private:
    static geometry_msgs::Vector3 replusive_forces(rigidbody* d, std::vector<rigidbody*>& rigidbodies);
    static geometry_msgs::Vector3 attractive_forces(rigidbody* d);
    static double distance_between(geometry_msgs::Point a, geometry_msgs::Point b);
    template <class T>
    static T difference(T a, T b);
    template<class T>
    static T add_vec3_or_point(T a, T b);
public:
    static bool check(rigidbody* d, std::vector<rigidbody*>& rigidbodies);


};


#endif //MULTI_DRONE_PLATFORM_POTENTIAL_FIELDS_H
