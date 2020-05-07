#include "rigidbody.h"

std::string get_pose_topic(const std::string& tag) {
    return "/vrpn_client_node/" + tag + "/pose";
}

double lerp(double begin, double end, double t) {
    return begin + (end - begin) * t;
}

double linear_lerp(double begin, double end, double maxChange) {
    double diff = end - begin;
    return begin + std::min(std::abs(diff), maxChange) * (std::signbit(diff)?-1.0:1.0);
}

// yaw (Z), pitch (Y), roll (X)
geometry_msgs::Quaternion to_quaternion(double yaw) {
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

class DRONE_WRAPPER(vflie, homePosX, homePosY)
    private:
    ros::Publisher posePub;
    ros::Publisher desPub;

    std::array<double, 3> positionArray = {{0.0, 0.0, 0.0}};
    std::array<double, 3> velocityArray = {{0.0, 0.0, 0.0}};
    std::array<double, 3> desiredVelocityArray = {{0.0, 0.0, 0.0}};
    std::array<double, 3> desiredPositionArray = {{0.0, 0.0, 0.0}};

    enum move_type {
        POSITION, VELOCITY
    } moveType = move_type::VELOCITY;
    
    double currentYaw = 0.0;
    double currentYawRate = 0.0;


    double endOfCommand = 0.0;

    double lastPoseUpdate = -1.0;

    void publish_current_pose() {
        geometry_msgs::Quaternion orientationVRPN = to_quaternion(this->currentYaw);

        geometry_msgs::PoseStamped translatedMsg;
        translatedMsg.pose.position.x = positionArray[1];
        translatedMsg.pose.position.y = positionArray[0] * -1;
        translatedMsg.pose.position.z = positionArray[2];

        translatedMsg.pose.orientation.x = orientationVRPN.y;
        translatedMsg.pose.orientation.y = orientationVRPN.x * -1;
        translatedMsg.pose.orientation.z = orientationVRPN.z;
        translatedMsg.pose.orientation.w = orientationVRPN.w;

        translatedMsg.header.stamp = ros::Time::now();
        translatedMsg.header.frame_id = "map";
        this->posePub.publish(translatedMsg);
    }

    void pub_des() {
        geometry_msgs::PoseStamped msg;
        msg.pose.position.x = this->desiredPose.position.x;
        msg.pose.position.y = this->desiredPose.position.y;
        msg.pose.position.z = this->desiredPose.position.z;
        msg.header.frame_id = "map";
        this->desPub.publish(msg);
    }

public:
    void on_init(std::vector<std::string> args) final {
        std::string desPoseTopic = "mdp/drone_" + std::to_string(this->get_id()) + "/des_pose";

        this->physical_limits.x = {{-3.0, 3.0}};
        this->physical_limits.y = {{-3.0, 3.0}};
        this->physical_limits.z = {{-3.0, 2.0}};

        this->posePub = this->droneHandle.advertise<geometry_msgs::PoseStamped> (get_pose_topic(this->get_tag()), 1);
        this->desPub = this->droneHandle.advertise<geometry_msgs::PoseStamped> (desPoseTopic, 1);
        // @TODO, add a unique home point system

        this->homePosition.x = std::stof(args[0]);
        this->homePosition.y = std::stof(args[1]);
        this->homePosition.z = 0.0;
        this->positionArray = {this->homePosition.x, this->homePosition.y, 0.0};

        this->publish_current_pose();
    };

    void on_deinit() final {}

    void on_set_position(geometry_msgs::Vector3 pos, 
                        float yaw,
                        float duration, 
                        bool isRelative) override {

        this->moveType = move_type::POSITION;

        if (isRelative) {
            pos.x = pos.x + positionArray[0];
            pos.y = pos.y + positionArray[1];
            pos.z = pos.z + positionArray[2];
        }

        this->desiredPositionArray[0] = pos.x;
        this->desiredPositionArray[1] = pos.y;
        this->desiredPositionArray[2] = pos.z;

        this->endOfCommand = ros::Time::now().toSec() + duration;
    }

    void on_set_velocity(geometry_msgs::Vector3 vel, float yawrate, float duration, bool relativeHeight) override {
        //@TODO: add relative height
        this->moveType = move_type::VELOCITY;
        this->endOfCommand = ros::Time::now().toSec() + duration;
    }

    void on_motion_capture(const geometry_msgs::PoseStamped& msg) final {

    }
    
    void on_update() override {
        if (lastPoseUpdate < 0.0) {lastPoseUpdate = ros::Time::now().toSec(); return;}
        double deltaTime = ros::Time::now().toSec() - lastPoseUpdate;
        double T = 10 * deltaTime;

        switch (moveType) {
            case move_type::POSITION: {

                std::array<double, 3> dirToDesPos = {{0.0, 0.0, 0.0}};
                dirToDesPos[0] = (this->desiredPositionArray[0] - this->positionArray[0]);
                dirToDesPos[1] = (this->desiredPositionArray[1] - this->positionArray[1]);
                dirToDesPos[2] = (this->desiredPositionArray[2] - this->positionArray[2]);

                double magnitude = std::abs(dirToDesPos[0]) + std::abs(dirToDesPos[1]) + std::abs(dirToDesPos[2]);
                if (magnitude > 0.0) {
                    double distanceToGoal = sqrt( (dirToDesPos[0]*dirToDesPos[0]) + (dirToDesPos[1]*dirToDesPos[1]) + (dirToDesPos[2]*dirToDesPos[2]) );

                    double timeLeft = endOfCommand - ros::Time::now().toSec();
                    double desiredVelocity = 0.0;
                    if (timeLeft > 0.0) {
                        // @FIX: this is probably good enough for now
                        // should be adjusted if we want more 'realistic' movement
                        desiredVelocity = 3*(distanceToGoal / timeLeft);
                    }

                    this->desiredVelocityArray[0] = (dirToDesPos[0] / magnitude) * desiredVelocity;
                    this->desiredVelocityArray[1] = (dirToDesPos[1] / magnitude) * desiredVelocity;
                    this->desiredVelocityArray[2] = (dirToDesPos[2] / magnitude) * desiredVelocity;
                } else {
                    this->desiredVelocityArray[0] = 0.0;
                    this->desiredVelocityArray[1] = 0.0;
                    this->desiredVelocityArray[2] = 0.0;
                }

                this->velocityArray[0] = linear_lerp(this->velocityArray[0], this->desiredVelocityArray[0], T);
                this->velocityArray[1] = linear_lerp(this->velocityArray[1], this->desiredVelocityArray[1], T);
                this->velocityArray[2] = linear_lerp(this->velocityArray[2], this->desiredVelocityArray[2], T);

            } break;
            case move_type::VELOCITY: {

                this->velocityArray[0] = linear_lerp(this->velocityArray[0], this->desiredVelocity.linear.x, T);
                this->velocityArray[1] = linear_lerp(this->velocityArray[1], this->desiredVelocity.linear.y, T);
                this->velocityArray[2] = linear_lerp(this->velocityArray[2], this->desiredVelocity.linear.z, T);

            } break;
            default: break;
        }

        this->positionArray[0] = this->positionArray[0] + (this->velocityArray[0] * deltaTime);
        this->positionArray[1] = this->positionArray[1] + (this->velocityArray[1] * deltaTime);
        this->positionArray[2] = this->positionArray[2] + (this->velocityArray[2] * deltaTime);

        this->publish_current_pose();
        this->pub_des();
        lastPoseUpdate = ros::Time::now().toSec();
    }

    void on_takeoff(float height, float duration) override {
        geometry_msgs::Vector3 pos;
        pos.x = this->currentPose.position.x;
        pos.y = this->currentPose.position.y;
        pos.z = height;

        on_set_position(pos, this->currentYaw, duration, false);
    }

    void on_land(float duration) override {
        geometry_msgs::Vector3 pos;
        pos.x = this->currentPose.position.x;
        pos.y = this->currentPose.position.y;
        pos.z = 0.0;

        on_set_position(pos, this->currentYaw, duration, false);
    }

    void on_emergency() override {
        geometry_msgs::Vector3 pos;
        pos.x = this->currentPose.position.x;
        pos.y = this->currentPose.position.y;
        pos.z = 0.0;

        on_set_position(pos, this->currentYaw, 0.5, false);
    }
};


