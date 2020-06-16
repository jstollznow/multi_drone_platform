#include "rigidbody.h"
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>

#define ICP_TEST false

std::string get_pose_topic(const std::string& tag) {
#if USE_NATNET
    return "/mocap/rigid_bodies/" + tag + "/pose";
#else
    return "/vrpn_client_node/" + tag + "/pose";
#endif
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
    Eigen::Quaterniond qE(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));

    geometry_msgs::Quaternion q;
    q.w = qE.w();
    q.x = qE.x();
    q.y = qE.y();
    q.z = qE.z();

    return q;
}

class DRONE_WRAPPER(vflie, homePosX, homePosY)
    private:
    ros::Publisher posePub;
#if ICP_TEST
    ros::Publisher markerPub;
#endif
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
        geometry_msgs::Quaternion orientation = to_quaternion(this->currentYaw);

        geometry_msgs::PoseStamped translatedMsg;
#if USE_NATNET
        // natnet is z up
        translatedMsg.pose.position.x = positionArray[0];
        translatedMsg.pose.position.y = positionArray[1];
        translatedMsg.pose.position.z = positionArray[2];

        translatedMsg.pose.orientation.x = orientation.x;
        translatedMsg.pose.orientation.y = orientation.y;
        translatedMsg.pose.orientation.z = orientation.z;
        translatedMsg.pose.orientation.w = orientation.w;
        translatedMsg.header.frame_id = "mocap";
#else /* USE VRPN */
        // convert to y up
        translatedMsg.pose.position.x = positionArray[1];
        translatedMsg.pose.position.y = positionArray[0] * -1;
        translatedMsg.pose.position.z = positionArray[2];

        translatedMsg.pose.orientation.x = orientation.y;
        translatedMsg.pose.orientation.y = orientation.x * -1;
        translatedMsg.pose.orientation.z = orientation.z;
        translatedMsg.pose.orientation.w = orientation.w;
        translatedMsg.header.frame_id = "map";
#endif
        translatedMsg.header.stamp = ros::Time::now();
        this->posePub.publish(translatedMsg);
    }

#if ICP_TEST
    /* publishes a marker set to emulate natnet point cloud, note this overrides a value from natnet and does not append to it. Only use this in testing */
    inline geometry_msgs::Point from_eigen(const Eigen::Vector3d& v) {
        geometry_msgs::Point r;
        r.x = v.x();
        r.y = v.y();
        r.z = v.z();
        return r;
    }

    void publish_vflie_marker_set() {
        std::array<Eigen::Vector3d, 4> markerTemplate = {{
                {0.04, 0.0, 0.0},
                {0.0, 0.04, 0.0},
                {0.0, -0.04, 0.0},
                {-0.04, 0.0, 0.04}
        }};
        Eigen::Quaterniond q(Eigen::AngleAxisd(sin(ros::Time::now().toSec()), Eigen::Vector3d::UnitZ()));
        for (Eigen::Vector3d& p : markerTemplate) {
            p = q * p;
            p += Eigen::Vector3d(positionArray[0], positionArray[1], positionArray[2]);
        }

        std::vector<geometry_msgs::Point> m;
        m.resize(markerTemplate.size());
        m[0] = from_eigen(markerTemplate[0]);
        m[1] = from_eigen(markerTemplate[1]);
        m[2] = from_eigen(markerTemplate[2]);
        m[3] = from_eigen(markerTemplate[3]);

        visualization_msgs::Marker ml;
        ml.header.frame_id = "world";
        ml.header.stamp = ros::Time::now();
        ml.points = m;
        ml.type = 7;
        ml.action = 0;
        geometry_msgs::Vector3 s; s.x = 0.01; s.y = 0.01; s.z = 0.01;
        ml.scale = s;
        ml.color.r = 1.0; ml.color.g = 1.0; ml.color.b = 1.0; ml.color.a = 1.0;
        markerPub.publish(ml);
    };
#endif /* ICP_TEST */

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

        this->posePub = this->droneHandle.advertise<geometry_msgs::PoseStamped> (get_pose_topic(this->get_tag()), 1);
        this->desPub = this->droneHandle.advertise<geometry_msgs::PoseStamped> (desPoseTopic, 1);
#if ICP_TEST
        this->markerPub = this->droneHandle.advertise<visualization_msgs::Marker> ("/markers/vis", 1);
#endif
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

#if ICP_TEST
        this->publish_vflie_marker_set(); // enable only when testing ICP
#else
        this->publish_current_pose();
#endif
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


