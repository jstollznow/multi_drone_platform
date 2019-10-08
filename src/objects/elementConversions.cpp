#include <string>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "multi_drone_platform/inputData.h"

namespace mdp_conversions
{
    struct euler_rotation
    {
        float Yaw;
        float Pitch;
        float Roll;
    };
    euler_rotation toEuler(geometry_msgs::Quaternion pQuaternion)
    {
        // do we need to consider gimbal lock case?

        euler_rotation angles;

        // roll
        double sinr_cosp = +2.0 * (pQuaternion.w * pQuaternion.x + pQuaternion.y * pQuaternion.z);
        double cosr_cosp = +1.0 - 2.0 * (pQuaternion.x * pQuaternion.x + pQuaternion.y * pQuaternion.y);
        angles.Roll = atan2(sinr_cosp, cosr_cosp);

        // pitch
        double sinp = +2.0 * (pQuaternion.w * pQuaternion.y - pQuaternion.z * pQuaternion.x);
        if (fabs(sinp) >= 1)
            angles.Pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            angles.Pitch = asin(sinp);

        // yaw
        double siny_cosp = +2.0 * (pQuaternion.w * pQuaternion.z + pQuaternion.x * pQuaternion.y);
        double cosy_cosp = +1.0 - 2.0 * (pQuaternion.y * pQuaternion.y + pQuaternion.z * pQuaternion.z);  
        angles.Yaw = atan2(siny_cosp, cosy_cosp);

        return angles;
    }
    geometry_msgs::Vector3 getUpVector(geometry_msgs::Quaternion pQuaternion)
    {
        geometry_msgs::Vector3 v;
        // UP (z)
        v.x = 2 * (pQuaternion.x*pQuaternion.z + pQuaternion.w*pQuaternion.y);
        v.y = 2 * (pQuaternion.y*pQuaternion.z - pQuaternion.w*pQuaternion.x);
        v.z = 1 - 2 * (pQuaternion.x*pQuaternion.x + pQuaternion.y*pQuaternion.y);
        return v;
    }
    float min(float a, float b)
    {
        return (a > b)? b: a;
    }
    geometry_msgs::Twist calcVel(geometry_msgs::PoseStamped& lastPos, geometry_msgs::PoseStamped& firstPos)
    {
        geometry_msgs::Twist returnVel;
        
        float dx = lastPos.pose.position.x - firstPos.pose.position.x;
        float dy = lastPos.pose.position.y - firstPos.pose.position.y;
        float dz = lastPos.pose.position.z - firstPos.pose.position.z;
        float dt = lastPos.header.stamp.sec - firstPos.header.stamp.sec;
        returnVel.linear.x =  dx / dt;
        returnVel.linear.y = dy / dt; 
        returnVel.linear.z = dz / dt;

        geometry_msgs::Vector3 lastPosAng = getUpVector(lastPos.pose.orientation);
        geometry_msgs::Vector3 firstPosAng = getUpVector(firstPos.pose.orientation);
        
        // not used by Duong in his algorithms
        // maybe yaw will be useful but pitch and roll will be internal controls
        
        
        // angular velocities
        // assume easiest route to the same point

        float rollDiff = (lastPosAng.x - firstPosAng.x);
        float pitchDiff = (lastPosAng.y - firstPosAng.y);
        float yawDiff = (lastPosAng.z - firstPosAng.z);

        // @FIX does not account for direction
        returnVel.angular.x =  min(rollDiff, 180 - rollDiff)/ dt;
        returnVel.angular.y = min(pitchDiff, 90 - pitchDiff) / dt;
        returnVel.angular.z = min(yawDiff, 180 - yawDiff) / dt;


        return returnVel;
    }
}