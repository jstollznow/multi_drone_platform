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


class cflie : public rigidbody {
    private:
    const std::string linkUri = "radio://0/80/2M";
    

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

    bool doOnce = false;

    void go_to(geometry_msgs::Vector3 goal, float yaw, float duration, bool isRelative) {
        crazyflie_driver::GoTo goToMsg;
        goToMsg.request.goal.x = goal.x;
        goToMsg.request.goal.y = goal.y;
        goToMsg.request.goal.z = goal.z;
        ROS_INFO("GOTO: [%f, %f, %f] in %f, relative: %d", goal.x, goal.y, goal.z, duration, isRelative);
        goToMsg.request.duration = ros::Duration(duration);
        goToMsg.request.yaw = yaw;
        goToMsg.request.relative = isRelative;
        
        auto startPoint = std::chrono::high_resolution_clock::now();
        if (!goToService.call(goToMsg)) {
            ROS_WARN("GOTO FAILED ON '%s'", tag.c_str());
            goToService = droneHandle.serviceClient<crazyflie_driver::GoTo>("/" + tag + "/go_to", true);
        }
        long long start = std::chrono::time_point_cast<std::chrono::milliseconds> (startPoint).time_since_epoch().count();
        long long end = std::chrono::time_point_cast<std::chrono::milliseconds> (std::chrono::high_resolution_clock::now()).time_since_epoch().count();
        ROS_WARN("GOTO TOOK %lld ms", end - start);

        reset_timeout(duration);
    }
    void battery_log(const std_msgs::Float32::ConstPtr &msg) {
        if (msg->data <= 3.15f) {
            ROS_WARN("Battery dying soon...");
            batteryDying = true;
        }
        else {
            batteryDying = false;
        }
    }
    public:
    cflie(std::string tag, uint32_t id):rigidbody(tag, id) {
        ROS_INFO("I am here, its %s", tag.c_str());
        droneAddress = (tag.substr(tag.find_first_of('_')+1, tag.length()));
        addCrazyflieService = droneHandle.serviceClient<crazyflie_driver::AddCrazyflie>("/add_crazyflie");
        crazyflie_driver::AddCrazyflie msg;
        msg.request.uri = linkUri + "/0xE7E7E7E7" + droneAddress;
        msg.request.tf_prefix = tag;
        msg.request.roll_trim = 0.0f;
        msg.request.pitch_trim = 0.0f;
        msg.request.enable_logging = false;
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
            ROS_INFO("%s launched on Crazyflie Server", tag.c_str());
        }
        else {
            ROS_ERROR("Could not add %s to Crazyflie Server, please check the drone tag", tag.c_str());
        }

        updateParams = droneHandle.serviceClient<crazyflie_driver::UpdateParams>("/" + tag + "/update_params");

        emergencyService = droneHandle.serviceClient<std_srvs::Empty>("/" + tag + "/emergency");        
        externalPosition = droneHandle.advertise<geometry_msgs::PointStamped>("/" + tag + "/external_position", 1);
        // high level commands
        takeoffService = droneHandle.serviceClient<crazyflie_driver::Takeoff>("/" + tag + "/takeoff");
        landService = droneHandle.serviceClient<crazyflie_driver::Land>("/" + tag + "/land");
        stopService = droneHandle.serviceClient<crazyflie_driver::Stop>("/" + tag + "/stop");
        goToService = droneHandle.serviceClient<crazyflie_driver::GoTo>("/" + tag + "/go_to", true);

        // feedback

        batteryCheck = droneHandle.subscribe<std_msgs::Float32>("/" + tag + "/battery", 10, &cflie::battery_log, this); 

    };

    ~cflie() {
        // use remove service, need to keep ros alive for a bit, ctrl c closes the crazyflie server before we have a chance to remove the crazyflie.
        // will need to make a workaround which keeps the server up long enough for us to remove the crazyflie...

        // auto RemCrazyflieService = droneHandle.serviceClient<crazyflie_driver::RemoveCrazyflie> ("/remove_crazyflie");
        // crazyflie_driver::RemoveCrazyflie msg;
        // msg.request.uri = link_uri + "/0xE7E7E7E7" + droneAddress;
        // if (RemCrazyflieService.call(msg)) {
        //     ROS_INFO("Removed %s from the crazyflie server", tag.c_str());
        // } else {
        //     ROS_INFO("Failed to remove %s from the crazyflie server", tag.c_str());
        // }
    }
    void on_motion_capture(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // external_pose.publish(msg);
        geometry_msgs::PointStamped pointMsg;
        pointMsg.header = msg->header;
        pointMsg.point = msg->pose.position;
        externalPosition.publish(pointMsg);
    }
    
    void on_update() override {
        if (doOnce == false) {
            droneHandle.setParam( "/" + tag + "/commander/enHighLevel", 1);
            droneHandle.setParam("/" + tag + "/stabilizer/estimator", 2);
            droneHandle.setParam( "/" + tag + "/stabilizer/controller", 2);
            droneHandle.setParam( "/" + tag + "/kalman/resetEstimation", 1);
            crazyflie_driver::UpdateParams paramsMsg;
            paramsMsg.request.params = {
                "commander/enHighLevel",
                "stabilizer/estimator",
                "stabilizer/controller",
                "kalman/resetEstimation"
            };
            if (updateParams.call(paramsMsg) == true) {
                doOnce = true;
            }
        }
        
    }

    void on_set_position(geometry_msgs::Vector3 pos, float yaw, float duration, bool isRelative) override {
        go_to(pos, yaw, duration, isRelative);
    }

    void on_set_velocity(geometry_msgs::Vector3 vel, float yawrate, float duration, bool isRelative) override {
        geometry_msgs::Vector3 positionGoal;
        positionGoal.x = (vel.x * duration);
        positionGoal.y = (vel.y * duration);
        positionGoal.z = (vel.z * duration);

        go_to(positionGoal, yawrate , duration, true);
    }

    void on_takeoff(float height, float duration) override {
        ROS_INFO("Takeoff sent to %s", tag.c_str());
        crazyflie_driver::Takeoff msg;
        msg.request.duration = ros::Duration(duration);
        msg.request.height = height;
        if (!takeoffService.call(msg)) {
            ROS_ERROR("Takeoff service failed");
        }
    }

    void on_land(float duration) override {
        ROS_INFO("Land sent to %s", tag.c_str());
        crazyflie_driver::Land msg;
        msg.request.duration = ros::Duration(duration);
        // 5cm above the ground?
        msg.request.height = 0.05f;
        if (!landService.call(msg)) {
            ROS_ERROR("Land service failed");
        }
    }

    void on_emergency() override {
        ROS_INFO("Emergency sent to %s", tag.c_str());
        std_srvs::Empty msg;
        if(emergencyService.call(msg)) {
            ROS_INFO("Emergency land for %s successful", tag.c_str());
        }
        else {
            ROS_INFO("Emergency land for %s not successful", tag.c_str());
        }
    }
};

