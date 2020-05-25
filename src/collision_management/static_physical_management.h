//
// Created by jacob on 29/4/20.
//


#ifndef MULTI_DRONE_PLATFORM_STATIC_PHYSICAL_MANAGEMENT_H
#define MULTI_DRONE_PLATFORM_STATIC_PHYSICAL_MANAGEMENT_H

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
class static_physical_management {

private:
    static std::array<double, 2> individual_velocity_boundaries(std::array<double, 2> limit, double currPos, double accel);
    static static_limits generate_velocity_boundaries(geometry_msgs::Vector3 currPos, double accel);
    static geometry_msgs::Vector3 predict_position(ros::Time lastUpdate, geometry_msgs::Twist currVel, geometry_msgs::Pose currPos, int timeSteps);
    static double predict_current_yaw(ros::Time lastUpdate, geometry_msgs::Twist currVel, geometry_msgs::Pose currPos, int timeSteps);
    static geometry_msgs::Vector3 vel_static_limits(rigidbody* d, geometry_msgs::Vector3 requestedVelocity);
    static geometry_msgs::Point pos_static_limits(rigidbody* d, geometry_msgs::Point requestedPos, double dur);
    static geometry_msgs::Vector3 check_physical_limits(rigidbody* d, geometry_msgs::Vector3 requestedVelocity);
    static geometry_msgs::Vector3 check_physical_limits(geometry_msgs::Vector3 requestedPosition);
public:
    static static_limits staticBoundary;
    static double adjust_for_physical_limits(rigidbody* d, geometry_msgs::Vector3& requestedPosition, double dur);
    static geometry_msgs::Vector3 adjust_for_physical_limits(rigidbody* d, geometry_msgs::Vector3 requestedVelocity);
    static multi_drone_platform::api_update adjust_command(rigidbody *d, const multi_drone_platform::api_update msg);
    static void check_land(rigidbody* d, multi_drone_platform::api_update& msg);
    static void check_go_home(rigidbody *d, multi_drone_platform::api_update &msg);
    static void make_absolute_velocity(rigidbody *d, multi_drone_platform::api_update &msg);
    static void make_absolute_position(rigidbody *d, multi_drone_platform::api_update &msg);



};


#endif //MULTI_DRONE_PLATFORM_STATIC_PHYSICAL_MANAGEMENT_H
