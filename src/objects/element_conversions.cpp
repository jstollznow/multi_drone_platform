#include <string>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

namespace mdp_conversions {

struct euler_rotation {
    float roll;
    float pitch;
    float yaw;
};

euler_rotation to_euler(geometry_msgs::Quaternion pQuaternion) {
    // do we need to consider gimbal lock case?

    euler_rotation angles;

    // roll
    double sinr_cosp = +2.0 * (pQuaternion.w * pQuaternion.x + pQuaternion.y * pQuaternion.z);
    double cosr_cosp = +1.0 - 2.0 * (pQuaternion.x * pQuaternion.x + pQuaternion.y * pQuaternion.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch
    double sinp = +2.0 * (pQuaternion.w * pQuaternion.y - pQuaternion.z * pQuaternion.x);
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // yaw
    double siny_cosp = +2.0 * (pQuaternion.w * pQuaternion.z + pQuaternion.x * pQuaternion.y);
    double cosy_cosp = +1.0 - 2.0 * (pQuaternion.y * pQuaternion.y + pQuaternion.z * pQuaternion.z);  
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}

geometry_msgs::Vector3 get_up_vector(geometry_msgs::Quaternion pQuaternion) {
    geometry_msgs::Vector3 v;
    // UP (z)
    v.x = 2 * (pQuaternion.x*pQuaternion.z + pQuaternion.w*pQuaternion.y);
    v.y = 2 * (pQuaternion.y*pQuaternion.z - pQuaternion.w*pQuaternion.x);
    v.z = 1 - 2 * (pQuaternion.x*pQuaternion.x + pQuaternion.y*pQuaternion.y);
    return v;
}

float min(float a, float b) {
    return (a > b)? b: a;
}

geometry_msgs::Twist calc_vel(geometry_msgs::PoseStamped& lastPos, geometry_msgs::PoseStamped& firstPos) {
    geometry_msgs::Twist returnVel;
    
    float dx = lastPos.pose.position.x - firstPos.pose.position.x;
    float dy = lastPos.pose.position.y - firstPos.pose.position.y;
    float dz = lastPos.pose.position.z - firstPos.pose.position.z;
    float dt = (lastPos.header.stamp.toSec() - firstPos.header.stamp.toSec());
    returnVel.linear.x =  dx / dt;
    returnVel.linear.y = dy / dt; 
    returnVel.linear.z = dz / dt;
    
    // ROS_INFO("%f / %f = %f", dx, dt, returnVel.linear.x);


    // // convert orientation to angular position
    // geometry_msgs::Vector3 lastPosAng = getUpVector(lastPos.pose.orientation);
    // geometry_msgs::Vector3 firstPosAng = getUpVector(firstPos.pose.orientation);
    
    // // not used by Duong in his algorithms
    // // maybe yaw will be useful but pitch and roll will be internal controls
    
    
    // // angular velocities
    // // assume easiest route to the same point

    // float rollDiff = (lastPosAng.x - firstPosAng.x);
    // float pitchDiff = (lastPosAng.y - firstPosAng.y);
    // float yawDiff = (lastPosAng.z - firstPosAng.z);

    // // @FIX: does not account for direction
    // returnVel.angular.x =  min(rollDiff, 180 - rollDiff)/ dt;
    // returnVel.angular.y = min(pitchDiff, 90 - pitchDiff) / dt;
    // returnVel.angular.z = min(yawDiff, 180 - yawDiff) / dt;


    return returnVel;
}

geometry_msgs::Vector3 point_to_vector3(geometry_msgs::Point& point) {
    geometry_msgs::Vector3 v;
    v.x = point.x;
    v.y = point.y;
    v.z = point.z;
    return v;
}
}
