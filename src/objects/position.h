#include <Eigen/Geometry>
#include "ros/ros.h"
#include "std_msgs/Header.h"
class position{
    public:
    std_msgs::Header posHeader;
    Eigen::Quaterniond wOrientation;
    Eigen::Vector3d world;
    float r;
    float p;
    float y;
    Eigen::VectorXd rot;
    // presuming ZYZ
    void defineRotate();
};