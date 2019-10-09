#include "velocity.h"
#include "position.h"
#include <string>
#include <vector>

Eigen::Vector3d velocity::globalToBody()
{

}
Eigen::Vector3d velocity::getEulerAngleDer()
{

}
Eigen::Vector3d velocity::getAngleRates(position& lastPos, position& firstPos)
{
    // this is the relative rotation from the first frame to the second frame
    relRotation = lastPos.wOrientation * firstPos.wOrientation.inverse();

    // XYZ convention, not sure if this is correct
    auto eulerDiff = relRotation.toRotationMatrix().eulerAngles(0,1,2);

    // thus, the eulerRates should be this diff/dt
    float dt = lastPos.posHeader.stamp.sec - firstPos.posHeader.stamp.sec;
    eulerAngleRate.x = eulerDiff.x / dt;
    eulerAngleRate.y = eulerDiff.y / dt;
    eulerAngleRate.z = eulerDiff.z / dt;
    
    auto angDiff = relRotation.toRotationMatrix();
    

}
void velocity::getAngularVelocity(position& lastPos)
{
    // http://andrew.gibiansky.com/downloads/pdf/Quadcopter%20Dynamics,%20Simulation,%20and%20Control.pdf 
    // https://liu.diva-portal.org/smash/get/diva2:1129641/FULLTEXT01.pdf 
    Eigen::VectorXd transform(3,3);
    transform <<1, 0, -sin(lastPos.p),
                0, cos(lastPos.r), cos(lastPos.p)*sin(lastPos.r),
                0, -sin(lastPos.r), cos(lastPos.p)*sin(lastPos.r);
    globalAngularRate = transform * eulerAngleRate;

}