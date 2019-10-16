#include "../objects/rigidBody.h"
#include "crazyflie_driver/AddCrazyflie.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"
#include "crazyflie_driver/Land.h"
#include "crazyflie_driver/GoTo.h"
#include "crazyflie_driver/Takeoff.h"
#include "crazyflie_driver/Hover.h"
#include "crazyflie_driver/Stop.h"
#include "crazyflie_driver/FullState.h"
#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/UpdateParams.h"
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


class cflie : public rigidBody
{
private:
    const std::string link_uri = "radio://0/80/2M";
    std::string droneAddress;
    ros::Publisher cmd_vel;
    ros::Publisher cmd_full_state;
    ros::Publisher external_position;
    ros::ServiceClient emergencyService;
    ros::Publisher cmd_hover;
    ros::Publisher cmd_stop;
    ros::Publisher cmd_position;

    // high level commands
    ros::ServiceClient takeoffService;
    ros::ServiceClient landService;
    ros::ServiceClient stopService;
    ros::ServiceClient goToService;
    ros::ServiceClient updateParams;
    ros::Subscriber imuMeasure;
    ros::ServiceClient addCrazyflieService;

    bool DoOnce = false;
    
    void postToCrazyServer()
    {
        // geometry_msgs::PointStamped myMsg;
        // myMsg.point = motionCapture.front().pose.position;
        // myMsg.header = motionCapture.front().header;
        // motionPub.publish(myMsg);
        
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // ROS_INFO("Some IMU data: \nAng Vel: %.6f, %.6f, %.6f\n", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    }

public:
    cflie(std::string tag):rigidBody(tag)
    {
        

        ROS_INFO("I am here, its %s", tag.c_str());
        droneAddress = (tag.substr(tag.find_first_of('_')+1, tag.length()));
        addCrazyflieService = droneHandle.serviceClient<crazyflie_driver::AddCrazyflie>("/add_crazyflie");
        crazyflie_driver::AddCrazyflie msg;
        msg.request.uri = link_uri + "/0xE7E7E7E7" + droneAddress;
        msg.request.tf_prefix = tag;
        msg.request.roll_trim = 0.0f;
        msg.request.pitch_trim = 0.0f;
        msg.request.enable_logging = true;
        msg.request.enable_parameters = true;
        msg.request.use_ros_time = true;
        msg.request.enable_logging_imu = true;
        msg.request.enable_logging_temperature = false;
        msg.request.enable_logging_magnetic_field = false;
        msg.request.enable_logging_pressure = false;
        msg.request.enable_logging_battery = false;
        msg.request.enable_logging_pose = false;
        msg.request.enable_logging_packets = false;

        if (addCrazyflieService.call(msg)) 
        {
            ROS_INFO("%s launched on Crazyflie Server", tag.c_str());
        }
        else
        {
            ROS_ERROR("Could not add %s to Crazyflie Server, please check the drone tag", tag.c_str());
        }
        // droneHandle.setParam( "/" + tag + "/commander/enHighLevel", 1);
        // droneHandle.setParam("/" + tag + "/stabilizer/estimator", 2);
        // droneHandle.setParam( "/" + tag + "/stabilizer/controller", 2);
        // droneHandle.setParam( "/" + tag + "/kalman/resetEstimation", 1);
        updateParams = droneHandle.serviceClient<crazyflie_driver::UpdateParams>("/cflie_00/update_params");
        // standard
        cmd_vel = droneHandle.advertise<geometry_msgs::Twist>("/" + tag + "/cmd_vel", DEFAULT_QUEUE);
        cmd_full_state = droneHandle.advertise<crazyflie_driver::FullState>("/" + tag + "/cmd_full_state",DEFAULT_QUEUE);
        external_position = droneHandle.advertise<geometry_msgs::PointStamped>("/" + tag + "/external_position", DEFAULT_QUEUE);
        emergencyService = droneHandle.serviceClient<std_srvs::Empty>("/" + tag + "/emergency");
        cmd_hover = droneHandle.advertise<crazyflie_driver::Hover>("/" + tag + "/cmd_hover", DEFAULT_QUEUE);
        cmd_stop = droneHandle.advertise<std_msgs::Empty>("/" + tag + "/cmd_stop", DEFAULT_QUEUE);
        cmd_position = droneHandle.advertise<crazyflie_driver::Position>("/" + tag + "/cmd_position", DEFAULT_QUEUE);

        // high level commands
        takeoffService = droneHandle.serviceClient<crazyflie_driver::Takeoff>("/" + tag + "/takeoff");
        landService = droneHandle.serviceClient<crazyflie_driver::Land>("/" + tag + "/land");
        stopService = droneHandle.serviceClient<crazyflie_driver::Stop>("/" + tag + "/stop");
        goToService = droneHandle.serviceClient<crazyflie_driver::GoTo>("/" + tag + "/go_to");

        // feedback
        imuMeasure = droneHandle.subscribe<sensor_msgs::Imu>("/" + tag + "/imu", 10, &cflie::imuCallback, this); 

        // m_serviceUpdateParams = n.advertiseService(m_tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);
        // crazyflie_driver::UpdateParams myMsg;
        // myMsg.request.params = {"/" + tag + "/commander/enHighLevel"};

    };
    ~cflie() 
    {
        // use remove service
    }
    
    void velocity(geometry_msgs::Vector3 vel, float duration) override
    {
        
    }
    void position(geometry_msgs::Point pos, float duration) override
    {
        geometry_msgs::Twist cmdVelMsg;
        cmdVelMsg.linear.x = 0;
        cmdVelMsg.linear.y = 0;
        cmdVelMsg.linear.z = desPos.position.z;
        cmdVelMsg.angular.z = 0;
        cmd_vel.publish(cmdVelMsg);

        // if (desPos.position.z == 0.0f)
        // {
        //     // crazyflie_driver::Land landMsg;
        //     // landMsg.request.height = 0.0;
        //     // landMsg.request.duration = ros::Duration(2.0);
        //     // landService.call(landMsg);

            
        //     cmdVelMsg.linear.x = 0;
        //     cmdVelMsg.linear.y = 0;
        //     cmdVelMsg.linear.z = 0;
        //     cmdVelMsg.angular.z = 0;
        // }
        // else if (desPos.position.z >= 1.0f)
        // {
            

        //     // crazyflie_driver::Takeoff myMsg;
        //     // myMsg.request.height = 1;
        //     // myMsg.request.duration = ros::Duration(4.0f);
        //     // takeoffService.call(myMsg);


        //     // crazyflie_driver::Hover hoverMsg;
        //     // hoverMsg.vx = 0;
        //     // hoverMsg.vy = 0;
        //     // hoverMsg.yawrate = 0;
        //     // hoverMsg.zDistance = 0.5;
        //     // // ROS_INFO("Hover published");
        //     // cmd_hover.publish(hoverMsg);
        // }
    }
    void wrapperControlLoop() override
    {
        // if (DoOnce == false) {
        //     droneHandle.setParam( "/" + tag + "/commander/enHighLevel", 1);
        //     droneHandle.setParam("/" + tag + "/stabilizer/estimator", 2);
        //     droneHandle.setParam( "/" + tag + "/stabilizer/controller", 2);
        //     droneHandle.setParam( "/" + tag + "/kalman/resetEstimation", 1);
        //     crazyflie_driver::UpdateParams paramsMsg;
        //     paramsMsg.request.params = {
        //         "commander/enHighLevel",
        //         "stabilizer/estimator",
        //         "stabilizer/controller",
        //         "kalman/resetEstimation"
        //     };
        //     if (updateParams.call(paramsMsg) == true) {
        //         DoOnce = true;
        //     }
        // }

        // postToCrazyServer();
        // ROS_INFO("dur: %f", commandDuration);
        position(currPos.position,0);
        if(commandDuration != 0.0f)
        {
            // ROS_INFO("Pos called from wrapper");
            
        }
    }
    void land() override
    {
        ROS_INFO("Land requested for %s", tag.c_str());
        crazyflie_driver::Land msg;
        msg.request.duration = ros::Duration(1.0f);
        // 5cm above the ground?
        msg.request.height = 0.05f;
        landService.call(msg);


    }
    void emergency() override
    {
        ROS_INFO("Emergency override requested for %s", tag.c_str());
        std_srvs::Empty msg;
        if(emergencyService.call(msg))
        {
            ROS_INFO("Emergency land for %s successful", tag.c_str());
        }
        else
        {
            ROS_INFO("Emergency land for %s not successful", tag.c_str());
        }
    }
};


