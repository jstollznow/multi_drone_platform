#include "rigidbody.h"
#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/RemoveCrazyflie.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "crazyflie_driver/Land.h"
#include "crazyflie_driver/GoTo.h"
#include "crazyflie_driver/Takeoff.h"
#include "crazyflie_driver/Hover.h"
#include "crazyflie_driver/Stop.h"
#include "crazyflie_driver/FullState.h"
#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/UpdateParams.h"

#include <chrono>
// Possible commands for crazyflie


/***************************************************************************************************************************************
        m_subscribeCmdVel = n.subscribe(m_tf_prefix + "/cmd_vel", 1, &CrazyflieROS::cmdVelChanged, this);
        m_subscribeCmdFullState = n.subscribe(m_tf_prefix + "/cmd_full_state", 1, &CrazyflieROS::cmdFullStateSetpoint, this);
        m_subscribeExternalPosition = n.subscribe(m_tf_prefix + "/external_position", 1, &CrazyflieROS::positionMeasurementChanged, this);
        m_subscribeExternalPose = n.subscribe(m_tf_prefix + "/external_pose", 1, &CrazyflieROS::poseMeasurementChanged, this);
        m_serviceEmergency = n.advertiseService(m_tf_prefix + "/emergency", &CrazyflieROS::emergency, this);
        m_subscribeCmdHover = n.subscribe(m_tf_prefix + "/cmd_hover", 1, &CrazyflieROS::cmdHoverSetpoint, this);
        m_subscribeCmdStop = n.subscribe(m_tf_prefix + "/cmd_stop", 1, &CrazyflieROS::cmdStop, this);
        m_subscribeCmdPosition = n.subscribe(m_tf_prefix + "/cmd_position", 1, &CrazyflieROS::cmdPositionSetpoint, this);


        m_serviceSetGroupMask = n.advertiseService(m_tf_prefix + "/set_group_mask", &CrazyflieROS::setGroupMask, this);
        m_serviceTakeoff = n.advertiseService(m_tf_prefix + "/takeoff", &CrazyflieROS::takeoff, this);
        m_serviceLand = n.advertiseService(m_tf_prefix + "/land", &CrazyflieROS::land, this);
        m_serviceStop = n.advertiseService(m_tf_prefix + "/stop", &CrazyflieROS::stop, this);
        m_serviceGoTo = n.advertiseService(m_tf_prefix + "/go_to", &CrazyflieROS::goTo, this);
        m_serviceUploadTrajectory = n.advertiseService(m_tf_prefix + "/upload_trajectory", &CrazyflieROS::uploadTrajectory, this);
        m_serviceStartTrajectory = n.advertiseService(m_tf_prefix + "/start_trajectory", &CrazyflieROS::startTrajectory, this);
***************************************************************************************************************************************/


class DRONE_WRAPPER(cflie, linkUri, droneAddress)
    private:
    const std::string linkUri = "radio://0/80/2M";
    std::string myUri;

    std::string droneAddress;
    ros::Publisher externalPosition;
    ros::ServiceClient emergencyService;

    // high level commands
    ros::ServiceClient takeoffService;
    ros::ServiceClient landService;
    ros::ServiceClient stopService;
    ros::ServiceClient goToService;
    
    ros::ServiceClient updateParams;
    ros::Subscriber batteryCheck;
    ros::ServiceClient addCrazyflieService;

    bool hasInitParams = false;

    void go_to(geometry_msgs::Vector3 goal, float yaw, float duration, bool isRelative) {
        crazyflie_driver::GoTo goToMsg;
        goToMsg.request.goal.x = goal.x;
        goToMsg.request.goal.y = goal.y;
        goToMsg.request.goal.z = goal.z;
        std::ostringstream logMsg;
        logMsg<<"GO_TO ["<<goal.x<<", "<<goal.y<<", "<<goal.z<<"] in "<<duration<<" relative: "<<isRelative;

        this->log(logger::DEBUG, logMsg.str());
        goToMsg.request.duration = ros::Duration(duration);
        goToMsg.request.yaw = yaw;
        goToMsg.request.relative = isRelative;
        
        auto startPoint = std::chrono::high_resolution_clock::now();

        if (!goToService.call(goToMsg)) {
            ROS_WARN("GOTO FAILED ON '%s'", this->get_tag().c_str());
            goToService = droneHandle.serviceClient<crazyflie_driver::GoTo>("/" + this->get_tag() + "/go_to", true);
        }
        long long start = std::chrono::time_point_cast<std::chrono::milliseconds> (startPoint).time_since_epoch().count();
        long long end = std::chrono::time_point_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now()).time_since_epoch().count();

        this->log(logger::WARN, "GO_TO TOOK "+ std::to_string(end - start) + " ms");
    }

    void battery_log(const std_msgs::Float32::ConstPtr &msg) {
        if (msg->data <= 3.15f) {
            this->log(logger::WARN, "Battery dying soon...");
            batteryDying = true;
        }
    }

    public:
    void on_init(std::vector<std::string> args) final {
        droneAddress = (this->get_tag().substr(this->get_tag().find_first_of('_')+1));
        addCrazyflieService = droneHandle.serviceClient<crazyflie_driver::AddCrazyflie>("/add_crazyflie");
        myUri = linkUri + "/0xE7E7E7E7" + droneAddress;

        crazyflie_driver::AddCrazyflie msg;
        msg.request.uri = myUri;
        msg.request.tf_prefix = this->get_tag();
        msg.request.roll_trim = 0.0f;
        msg.request.pitch_trim = 0.0f;
        msg.request.enable_logging = true;
        msg.request.enable_parameters = true;
        msg.request.use_ros_time = true;
        msg.request.enable_logging_imu = false;
        msg.request.enable_logging_temperature = false;
        msg.request.enable_logging_magnetic_field = false;
        msg.request.enable_logging_pressure = false;
        msg.request.enable_logging_battery = true;
        msg.request.enable_logging_pose = false;
        msg.request.enable_logging_packets = false;

        if (addCrazyflieService.call(msg)) {
            this->log(logger::INFO, "Launched on Crazyflie Server");
        } else {
            this->log(logger::ERROR, "Could not add to Crazyflie Server, please check the drone tag");
        }

        updateParams = droneHandle.serviceClient<crazyflie_driver::UpdateParams>("/" + this->get_tag() + "/update_params");
        emergencyService = droneHandle.serviceClient<std_srvs::Empty>("/" + this->get_tag() + "/emergency");
        externalPosition = droneHandle.advertise<geometry_msgs::PointStamped>("/" + this->get_tag() + "/external_position", 1);

        // high level commands
        takeoffService = droneHandle.serviceClient<crazyflie_driver::Takeoff>("/" + this->get_tag() + "/takeoff");
        landService = droneHandle.serviceClient<crazyflie_driver::Land>("/" + this->get_tag() + "/land");
        stopService = droneHandle.serviceClient<crazyflie_driver::Stop>("/" + this->get_tag() + "/stop");
        goToService = droneHandle.serviceClient<crazyflie_driver::GoTo>("/" + this->get_tag() + "/go_to", true);

        // feedback
        batteryCheck = droneHandle.subscribe<std_msgs::Float32>("/" + this->get_tag() + "/battery", 10, &cflie::battery_log, this);

    };

    void on_deinit() final {
        std::string tag = this->get_tag();
        /* remove the crazyflie from the crazyserver */
        auto removeService = droneHandle.serviceClient<crazyflie_driver::RemoveCrazyflie> ("/remove_crazyflie");

        crazyflie_driver::RemoveCrazyflie msg;
        msg.request.uri = myUri;

        if (removeService.call(msg)) {
            this->log(logger::INFO, "Removed " + this->get_tag() + " from the crazyflie server");
        } else {
            this->log(logger::WARN, "Failed to remove " + this->get_tag() + " from the crazyflie server");
        }
    }
    
    void on_motion_capture(const geometry_msgs::PoseStamped& msg) override {
        // external_pose.publish(msg);
        geometry_msgs::PointStamped pointMsg;
        pointMsg.header = msg.header;
        pointMsg.point = msg.pose.position;
        externalPosition.publish(pointMsg);
    }
    
    void on_update() override {
        if (!hasInitParams) {
            droneHandle.setParam( "/" + this->get_tag() + "/commander/enHighLevel", 1);
            droneHandle.setParam("/" + this->get_tag() + "/stabilizer/estimator", 2);
            droneHandle.setParam( "/" + this->get_tag() + "/stabilizer/controller", 2);
            droneHandle.setParam( "/" + this->get_tag() + "/kalman/resetEstimation", 1);
            crazyflie_driver::UpdateParams paramsMsg;
            paramsMsg.request.params = {
                "commander/enHighLevel",
                "stabilizer/estimator",
                "stabilizer/controller",
                "kalman/resetEstimation"
            };

            hasInitParams = updateParams.call(paramsMsg);
        }
    }

    void on_set_position(geometry_msgs::Vector3 pos, float yaw, float duration, bool isRelative) override {
        go_to(pos, yaw, duration, isRelative);
    }

    void on_set_velocity(geometry_msgs::Vector3 vel, float yawrate, float duration, bool relativeHeight) override {
        geometry_msgs::Vector3 positionGoal;
        positionGoal.x = (vel.x * duration);
        positionGoal.y = (vel.y * duration);
        positionGoal.z = (vel.z * duration);

        go_to(positionGoal, yawrate , duration, true);
    }

    void on_takeoff(float height, float duration) override {
        this->log(logger::INFO, "Takeoff requested");
        crazyflie_driver::Takeoff msg;
        msg.request.duration = ros::Duration(duration);
        msg.request.height = height;
        if (!takeoffService.call(msg)) {
            this->log(logger::ERROR, "Takeoff service failed");
        }
    }

    void on_land(float duration) override {
        this->log(logger::INFO, "Land requested");
        crazyflie_driver::Land msg;
        msg.request.duration = ros::Duration(duration);
        // 5cm above the ground?
        msg.request.height = 0.05f;
        if (!landService.call(msg)) {
            this->log(logger::ERROR, "Land service failed");
        }
    }

    void on_emergency() override {
        this->log(logger::INFO, "Emergency requested");
        std_srvs::Empty msg;
        if(emergencyService.call(msg)) {
            this->log(logger::INFO, "Emergency land successful");
        } else {
            this->log(logger::INFO, "Emergency land was not successful");
        }
    }
};


