#include <string>
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace mdp_conversions {

struct euler_rotation {
    float roll;
    float pitch;
    float yaw;
};

/**
 * converts a quaternion into euler orientations
 * @param pQuaternion the input quaternion
 * @return a structure holding yaw, pitch, and roll rotations
 */
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

/**
 * returns the up vector given a quaternion rotation
 * @param pQuaternion the quaternion
 * @return a unit vector 3 holding the up direction
 */
geometry_msgs::Vector3 get_up_vector(geometry_msgs::Quaternion pQuaternion) {
    geometry_msgs::Vector3 v;
    // UP (z)
    v.x = 2 * (pQuaternion.x * pQuaternion.z + pQuaternion.w * pQuaternion.y);
    v.y = 2 * (pQuaternion.y * pQuaternion.z - pQuaternion.w * pQuaternion.x);
    v.z = 1 - 2 * (pQuaternion.x * pQuaternion.x + pQuaternion.y * pQuaternion.y);
    return v;
}

/**
 * returns the minimum of two functions... why didnt we just use std::min(..)?
 * @param a float a
 * @param b float b
 * @return the minimum of the two
 */
float min(float a, float b) {
    return (a > b) ? b : a;
}

/**
 * calculates the velocity between two PoseStampeds
 * @param newestPose the earlier of the two poses
 * @param oldestPose the later of the two poses
 * @return the velocity between them
 */
geometry_msgs::Twist calc_vel(geometry_msgs::PoseStamped &newestPose, geometry_msgs::PoseStamped &oldestPose) {
    geometry_msgs::Twist returnVel;

    float dx = newestPose.pose.position.x - oldestPose.pose.position.x;
    float dy = newestPose.pose.position.y - oldestPose.pose.position.y;
    float dz = newestPose.pose.position.z - oldestPose.pose.position.z;
    float dt = (newestPose.header.stamp.toSec() - oldestPose.header.stamp.toSec());

    if (dt == 0.0) {
        dt = 1.0;
    }

    returnVel.linear.x = dx / dt;
    returnVel.linear.y = dy / dt;
    returnVel.linear.z = dz / dt;

    // ROS_INFO("%f / %f = %f", dx, dt, returnVel.linear.x);


     // convert orientation to angular position
     auto lastPosEuler = to_euler(newestPose.pose.orientation);
     auto firstPosEuler = to_euler(oldestPose.pose.orientation);

     // not used by Duong in his algorithms
     // maybe yaw will be useful but pitch and roll will be internal controls


     // angular velocities
     // assume easiest route to the same point

     float rollDiff = (lastPosEuler.roll - firstPosEuler.roll);
     float pitchDiff = (lastPosEuler.pitch - firstPosEuler.pitch);
     float yawDiff = (lastPosEuler.yaw - firstPosEuler.yaw);

     // @FIX: does not account for direction
     returnVel.angular.x =  min(rollDiff, 360.0 - rollDiff)/ dt;
     returnVel.angular.y = min(pitchDiff, 180.0 - pitchDiff) / dt;
     returnVel.angular.z = min(yawDiff, 360.0 - yawDiff) / dt;
    return returnVel;
}

/**
 * converts a ROS point to a ROS vector3
 * @param point the geometry_msgs::Point
 * @return the geometry_msgs::Vector3
 */
geometry_msgs::Vector3 point_to_vector3(geometry_msgs::Point &point) {
    geometry_msgs::Vector3 v;
    v.x = point.x;
    v.y = point.y;
    v.z = point.z;
    return v;
}

/**
 * converts radians to degrees
 * @param rads r
 * @return deg
 */
constexpr float to_degrees(float rads) {
    return rads * 57.2957795f;
}

/**
 * converts degrees to radians
 * @param degrees deg
 * @return r
 */
constexpr float to_rads(float degrees) {
    return degrees / 57.2957795f;
}

/**
 * returns the yaw from a given pose
 * @param pos the input pose
 * @return the yaw of that pose in degrees
 */
float get_yaw_from_pose(const geometry_msgs::Pose &pos) {
    return to_degrees(to_euler(pos.orientation).yaw);
}

}
