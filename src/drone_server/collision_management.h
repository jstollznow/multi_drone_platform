//
// Created by jacob on 29/4/20.
//


#ifndef MULTI_DRONE_PLATFORM_COLLISION_MANAGEMENT_H
#define MULTI_DRONE_PLATFORM_COLLISION_MANAGEMENT_H

#include "rigidbody.h"

using coord_array = std::array<double, 3>;
struct static_limits {
//        std::array<double, 2> x = {{-1.60, 0.95}};
//        std::array<double, 2> y = {{-1.30, 1.30}};
//        std::array<double, 2> z = {{ 0.10, 1.80}};

// for vflie
    std::array<double, 2> x;
    std::array<double, 2> y;
    std::array<double, 2> z;
    static_limits(std::array<double, 2> x, std::array<double, 2> y, std::array<double, 2> z) :
    x(x), y(y), z(z) {}

    static_limits() {
        x = {{0.0, 0.0}};
        y = {{0.0, 0.0}};
        z = {{0.0, 0.0}};
    }

};
class collision_management {

private:


//    std::map<int, coord_array> positionLimitations;
//    std::map<int, coord_array> dynamicLimitations;
//    coord_array predict_velocity(coord_array currVelocity, coord_array desiredVelocity);
    static std::array<double, 2> individual_velocity_boundaries(std::array<double, 2> limit, double currPos, double accel);
    static static_limits generate_velocity_boundaries(geometry_msgs::Vector3 currPos, double accel);
    static geometry_msgs::Vector3 predict_position(ros::Time lastUpdate, geometry_msgs::Twist currVel, geometry_msgs::Pose currPos, int timeSteps);

public:
    static static_limits staticBoundary;
    static geometry_msgs::Vector3 check_static_limits(rigidbody* d, geometry_msgs::Vector3 requestedVelocity);
};


#endif //MULTI_DRONE_PLATFORM_COLLISION_MANAGEMENT_H
