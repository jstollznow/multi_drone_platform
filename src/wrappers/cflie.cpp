#include "../objects/rigidBody.h"
#include "crazyflie_driver/AddCrazyflie.h"
#include "std_srvs/Empty.h"
#include "crazyflie_driver/Land.h"
#include "crazyflie_driver/GoTo.h"

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
    ros::Subscriber imuMeasure;
    ros::ServiceClient addCrazyflieService;
    ros::ServiceClient emergencyService;
    ros::ServiceClient landService;
    ros::ServiceClient goToService;


    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        ROS_INFO("Some IMU data: \nAng Vel: %.6f, %.6f, %.6f\n", msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
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
        msg.request.enable_parameters = false;
        msg.request.use_ros_time = false;
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

        imuMeasure = droneHandle.subscribe<sensor_msgs::Imu>(tag + "/imu", 10, &cflie::imuCallback, this); 
        
        emergencyService = droneHandle.serviceClient<std_srvs::Empty>("/emergency");

        landService = droneHandle.serviceClient<crazyflie_driver::Land>("/land");

        goToService = droneHandle.serviceClient<crazyflie_driver::GoTo>("/go_to");
    };
    ~cflie() 
    {
        
    }
    
    void velocity(geometry_msgs::Vector3 vel, float duration) override
    {
        
    }
    void position(geometry_msgs::Point pos, float duration) override
    {
        // m_serviceGoTo = n.advertiseService(m_tf_prefix + "/go_to", &CrazyflieROS::goTo, this);
        crazyflie_driver::GoTo msg;
        msg.request.duration = ros::Duration(duration);
        msg.request.goal = pos;
        // relative position I think? as in a movement for the drone, not in world coords
        msg.request.relative = true;

    }
    void wrapperControlLoop() override
    {
        // implemented later
        // or not 
    }
    void land() override
    {
        ROS_INFO("Land requested for %s", tag.c_str());
        crazyflie_driver::Land msg;
        msg.request.duration = ros::Duration(1.0f);
        // 5cm above the ground?
        msg.request.height = 0.05f;

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


