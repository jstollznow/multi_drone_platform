#include <Eigen/Geometry>
class velocity{
    public:
        // obtained from motion capture
        Eigen::Vector3d globalLinear;
        Eigen::Vector3d globalAngularRate;


        // calculated
        Eigen::Quaterniond relRotation;
        Eigen::Vector3d eulerAngleRate;
        Eigen::Vector3d angVel;
        
        float rollR;
        float pitchR;
        float yawR;
        
        // Euler angle derivatives

        // according to Rotation Matrix
        Eigen::Vector3d globalToBody();

        Eigen::Vector3d getEulerAngleDer();
        Eigen::Vector3d getAngleRates(position& lastPos, position& firstPos);

        void getAngularVelocity(position& lastPos);

};