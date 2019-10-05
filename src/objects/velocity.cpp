#include <string>
#include <vector>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "multi_drone_platform/inputData.h"

namespace mdp_velControl
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
    multi_drone_platform::inputData calcVel(geometry_msgs::PoseStamped lastPos, geometry_msgs::PoseStamped firstPos)
    {
        multi_drone_platform::inputData currVel;
        float dx = lastPos.pose.position.x - firstPos.pose.position.x;
        float dy = lastPos.pose.position.y - firstPos.pose.position.y;
        float dz = lastPos.pose.position.z - firstPos.pose.position.z;
        float dt = lastPos.header.stamp.sec - firstPos.header.stamp.sec;
        currVel.posvel.x =  dx / dt;
        currVel.posvel.y = dy / dt; 
        currVel.posvel.z = dz / dt;

        geometry_msgs::Vector3 lPosAVel = getUpVector(lastPos.pose.orientation);
        geometry_msgs::Vector3 fPosAVel = getUpVector(firstPos.pose.orientation);
        
        // angular velocities
        currVel.forward.x = (lPosAVel.x - fPosAVel.x) / dt;
        currVel.forward.y = (lPosAVel.y - fPosAVel.y) / dt;
        currVel.forward.z = (lPosAVel.z - fPosAVel.z) / dt;
    }
}