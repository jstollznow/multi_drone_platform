#include "../objects/rigidBody.h"

std::string getPoseTopic(std::string& tag)
{
    return "/vrpn_client_node/" + tag + "/pose";
}

double lerp(double begin, double end, double t)
{
    return begin + (end - begin) * t;
}

double linearLerp(double begin, double end, double maxChange)
{
    double diff = end - begin;
    return begin + std::min(std::abs(diff), maxChange) * (std::signbit(diff)?-1.0:1.0);
}

geometry_msgs::Quaternion ToQuaternion(double yaw) // yaw (Z), pitch (Y), roll (X)
{
    yaw = yaw * 0.01745; // to radians
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    geometry_msgs::Quaternion q;
    q.w = cy;
    q.x = cy;
    q.y = 0.0;
    q.z = sy;

    return q;
}

class vflie : public rigidBody
{
private:

    ros::Publisher PosePub;
    ros::Publisher DesPub;

    std::array<double, 3> PositionArray = {{0.0, 0.0, 0.0}};
    std::array<double, 3> VelocityArray = {{0.0, 0.0, 0.0}};
    std::array<double, 3> DesiredVelocityArray = {{0.0, 0.0, 0.0}};
    std::array<double, 3> DesiredPositionArray = {{0.0, 0.0, 0.0}};

    enum MOVETYPE {
        POSITION, VELOCITY
    } MoveType = MOVETYPE::VELOCITY;
    
    double Yaw = 0.0;
    double YawRate = 0.0;


    double EndOfCommand = 0.0;

    double LastPoseUpdate = -1.0;

    void publishCurrentPose()
    {
        geometry_msgs::PoseStamped msg;
        msg.pose.position.x = PositionArray[0];
        msg.pose.position.y = PositionArray[1];
        msg.pose.position.z = PositionArray[2];
        msg.pose.orientation = ToQuaternion(this->Yaw);

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        this->PosePub.publish(msg);
    }

    void pubDes()
    {
        geometry_msgs::PoseStamped msg;
        msg.pose.position.x = this->DesiredPose.position.x;
        msg.pose.position.y = this->DesiredPose.position.y;
        msg.pose.position.z = this->DesiredPose.position.z;
        msg.header.frame_id = "map";
        this->DesPub.publish(msg);
    }

public:
    vflie(std::string tag, uint32_t id) : rigidBody(tag, id)
    {
        this->PosePub = this->droneHandle.advertise<geometry_msgs::PoseStamped> (getPoseTopic(tag), 1);
        this->DesPub = this->droneHandle.advertise<geometry_msgs::PoseStamped> ("/despos", 1);
        // @TODO, add a unique home point system
        
        int hpx = 0;
        if (tag == "vflie_00") {
            hpx = -1.0;
        } else if (tag == "vflie_01") {
            hpx = 1.0;
        } else if (tag == "vflie_02") {
            hpx = 0.5;
        } else if (tag == "vflie_03") {
            hpx = -0.5;
        }
        this->HomePosition.x = -1.0;
        this->HomePosition.y = hpx;
        this->HomePosition.z = 0.0;
        this->PositionArray = {-1.0, hpx, 0.0};

        this->publishCurrentPose();
    };

    ~vflie() 
    {

    }

    void onSetPosition(geometry_msgs::Vector3 pos, float yaw, float duration, bool isRelative) override
    {
        this->MoveType = MOVETYPE::POSITION;

        if (isRelative) {
            pos.x = pos.x + PositionArray[0];
            pos.y = pos.y + PositionArray[1];
            pos.z = pos.z + PositionArray[2];
        }

        this->DesiredPositionArray[0] = pos.x;
        this->DesiredPositionArray[1] = pos.y;
        this->DesiredPositionArray[2] = pos.z;

        this->EndOfCommand = ros::Time::now().toSec() + duration;
    }

    void onSetVelocity(geometry_msgs::Vector3 vel, float yawrate, float duration, bool isRelative) override
    {
        this->MoveType = MOVETYPE::VELOCITY;
        this->EndOfCommand = ros::Time::now().toSec() + duration;
    }

    void onMotionCapture(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
    }
    
    void onUpdate() override
    {
        if (LastPoseUpdate < 0.0) {LastPoseUpdate = ros::Time::now().toSec(); return;}
        double deltaTime = ros::Time::now().toSec() - LastPoseUpdate;
        double T = 2 * deltaTime;

        switch (MoveType) {
            case MOVETYPE::POSITION: {

                std::array<double, 3> DirToDesPos = {{0.0, 0.0, 0.0}};
                DirToDesPos[0] = (this->DesiredPositionArray[0] - this->PositionArray[0]);
                DirToDesPos[1] = (this->DesiredPositionArray[1] - this->PositionArray[1]);
                DirToDesPos[2] = (this->DesiredPositionArray[2] - this->PositionArray[2]);

                double Magnitude = std::abs(DirToDesPos[0]) + std::abs(DirToDesPos[1]) + std::abs(DirToDesPos[2]);
                if (Magnitude > 0.0) {
                    double DistanceToGoal = sqrt( (DirToDesPos[0]*DirToDesPos[0]) + (DirToDesPos[1]*DirToDesPos[1]) + (DirToDesPos[2]*DirToDesPos[2]) );

                    double TimeLeft = EndOfCommand - ros::Time::now().toSec();
                    double DesiredVelocity = 0.0;
                    if (TimeLeft > 0.0) {
                        // @FIX: this is probably good enough for now
                        // should be adjusted if we want more 'realistic' movement
                        DesiredVelocity = 3*(DistanceToGoal / TimeLeft);
                    }

                    this->DesiredVelocityArray[0] = (DirToDesPos[0] / Magnitude) * DesiredVelocity;
                    this->DesiredVelocityArray[1] = (DirToDesPos[1] / Magnitude) * DesiredVelocity;
                    this->DesiredVelocityArray[2] = (DirToDesPos[2] / Magnitude) * DesiredVelocity;
                } else {
                    this->DesiredVelocityArray[0] = 0.0;
                    this->DesiredVelocityArray[1] = 0.0;
                    this->DesiredVelocityArray[2] = 0.0;
                }

                this->VelocityArray[0] = linearLerp(this->VelocityArray[0], this->DesiredVelocityArray[0], T);
                this->VelocityArray[1] = linearLerp(this->VelocityArray[1], this->DesiredVelocityArray[1], T);
                this->VelocityArray[2] = linearLerp(this->VelocityArray[2], this->DesiredVelocityArray[2], T);

            } break;
            case MOVETYPE::VELOCITY: {

                this->VelocityArray[0] = linearLerp(this->VelocityArray[0], this->DesiredVelocity.linear.x, T);
                this->VelocityArray[1] = linearLerp(this->VelocityArray[1], this->DesiredVelocity.linear.y, T);
                this->VelocityArray[2] = linearLerp(this->VelocityArray[2], this->DesiredVelocity.linear.z, T);

            } break;
            default: break;
        }

        this->PositionArray[0] = this->PositionArray[0] + (this->VelocityArray[0] * deltaTime);
        this->PositionArray[1] = this->PositionArray[1] + (this->VelocityArray[1] * deltaTime);
        this->PositionArray[2] = this->PositionArray[2] + (this->VelocityArray[2] * deltaTime);

        this->publishCurrentPose();
        this->pubDes();
        LastPoseUpdate = ros::Time::now().toSec();
    }

    void onTakeoff(float height, float duration) override
    {
        geometry_msgs::Vector3 pos;
        pos.x = this->CurrentPose.position.x;
        pos.y = this->CurrentPose.position.y;
        pos.z = height;

        onSetPosition(pos, this->Yaw, duration, false);
    }

    void onLand(float duration) override
    {
        geometry_msgs::Vector3 pos;
        pos.x = this->CurrentPose.position.x;
        pos.y = this->CurrentPose.position.y;
        pos.z = 0.0;

        onSetPosition(pos, this->Yaw, duration, false);
    }

    void onEmergency() override
    {
        geometry_msgs::Vector3 pos;
        pos.x = this->CurrentPose.position.x;
        pos.y = this->CurrentPose.position.y;
        pos.z = 0.0;

        onSetPosition(pos, this->Yaw, 0.5, false);
    }
};


