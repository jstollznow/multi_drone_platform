#include "position.h"

void position::defineRotate()
{
    // r is roll
    // p is pitch
    // y is yaw


    // tait bryan angles
    Eigen::VectorXd Ry = Eigen::VectorXd(3,3);
    Ry <<
    cos(y),     sin(y),     0,
    -sin(y),    cos(y),     0,
    0,          0,          1;

    Eigen::VectorXd Rp = Eigen::VectorXd(3,3);
    Rp <<
    cos(p),     0,          -sin(p),
    0,          1,          0,
    sin(p),     0,          cos(p);

    Eigen::VectorXd Rr = Eigen::VectorXd(3,3);
    Rr<<
    1,          0,          0,
    0,          cos(r),     sin(r),
    0,          -sin(r),    cos(r);


    // RRset = {Rr, Rp, Ry};
    // orders = [...
    // 1,2,3;...
    // 1,3,2;...
    // 2,1,3;...
    // 2,3,1;...
    // 3,1,2;...
    // 3,2,1;...
    // ];
    rot = Rr * Rp * Ry;
    
}