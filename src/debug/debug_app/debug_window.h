#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/PoseArray.h>
#include "multi_drone_platform/log.h"
#include "gtk_ref.h"
#include "user_api.h"

#define UPDATE_RATE 10
#define LOG_POST_RATE 500
#define UI_PATH "/src/debug/debug_app/debug.ui"
#define SESSION_PARAM "/mdp/session_directory"
class debug_window: public Gtk::Window {
public:
    ros::AsyncSpinner windowSpinner;

    debug_window(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade);
    void init(mdp::id droneName, std::array<int, 2> startLocation = {0, 0}, bool expanded = false);
    void update_ui_labels();
private:
    // functions
    std::string round_to_string(double val, int n);
    void fetch_state_param();
    bool ros_spin();
    void update_ui_on_resume();
    void link_widgets();
    void on_landButton_clicked();
    void on_emergencyButton_clicked();
    void on_speedScale_value_changed();
    void on_expandButton_clicked();
    void write_to_file();
    void draw_obstacles();
    bool on_close(GdkEventAny* event);
    void obstacle_callback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void log_callback(const multi_drone_platform::log::ConstPtr& msg);
    void curr_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void des_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void curr_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void des_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
    void battery_callback(const std_msgs::Float32::ConstPtr& msg);

    void fill_panel(geometry_msgs::Pose ob, int level);
    std::vector<int> reduce(std::vector<int> a, std::vector<int> b);
    void check_x(geometry_msgs::Pose ob, std::vector<int>& candidates);
    void check_y(geometry_msgs::Pose ob, std::vector<int>& candidates);
    void reset_all_panels();
    bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override;
    // data

    // ROS related
    ros::CallbackQueue windowQueue;

    mdp::id myDrone;
    double dWidth;
    double dHeight;
    double dLength;
    double dRestrictedDistance;
    double dInfluenceDistance;

    ros::NodeHandle windowNode;
    ros::Subscriber logSubscriber;
    ros::Subscriber currTwistSubscriber;
    ros::Subscriber currPoseSubscriber;
    ros::Subscriber desPoseSubscriber;
    ros::Subscriber desTwistSubscriber;
    ros::Subscriber obstacleSubscriber;
    ros::Subscriber batterySubscriber;

    geometry_msgs::PoseStamped currPositionMsg;
    geometry_msgs::PoseStamped desPositionMsg;

    geometry_msgs::TwistStamped currVelocityMsg;
    geometry_msgs::TwistStamped desVelocityMsg;

    geometry_msgs::PoseArray obstacles;

    double batteryPercent;

    // internal variables
    std::string toAddToLog;
    std::string currState;
    bool expanded;
    double droneSpeedMultiplier;
    bool first;
    double maxMag;
    ros::Time firstTimeStamp;

    // UI Related
    Glib::Dispatcher dispatcher;
    Glib::RefPtr<Gtk::Builder> builder;
    Gtk::Label* droneNameLabel;
    std::string logTopic;
    double inner = 0.3;
    double middle = 0.6;
    double outer = 0.9;
    double lineWidth = 0.1;
    std::array<double,3> defaultColor = std::array<double,3> {0.5, 0.8, 1.0};

    // velocity/position labels
    Gtk::Label* currPosX;
    Gtk::Label* currPosY;
    Gtk::Label* currPosZ;
    Gtk::Label* currYaw;

    Gtk::Label* currVelX;
    Gtk::Label* currVelY;
    Gtk::Label* currVelZ;
    Gtk::Label* currYawRate;

    Gtk::Label* desPosX;
    Gtk::Label* desPosY;
    Gtk::Label* desPosZ;
    Gtk::Label* desYaw;

    Gtk::Label* desVelX;
    Gtk::Label* desVelY;
    Gtk::Label* desVelZ;
    Gtk::Label* desYawRate;

    // extra information
    Gtk::Label* stateInput;
    Gtk::LevelBar* batteryLevelBar;
    Gtk::Label* speedLabel;
    Gtk::Label* maxSpeedLabel;
    Gtk::Label* pktLossLabel;

    // logging
    Gtk::TextView* logTextView;
    Glib::RefPtr<Gtk::TextBuffer> logTextBuffer;
    Gtk::ScrolledWindow* logScroll;

    // for user interaction
    Gtk::Button* landButton;
    Gtk::Button* emergencyButton;
    Gtk::Button* expandButton;
    Gtk::Image* compressImage;
    Gtk::Image* expandImage;
    Gtk::Grid* sidePanelGrid;

    // collisionAvoidanceDrawingAreas
    Gtk::DrawingArea* topViewBottom;
    Gtk::DrawingArea* topViewTop;
    Gtk::DrawingArea* topViewLeft;
    Gtk::DrawingArea* topViewRight;

    Gtk::DrawingArea* topViewTopLeft;
    Gtk::DrawingArea* topViewTopRight;
    Gtk::DrawingArea* topViewBotRight;
    Gtk::DrawingArea* topViewBotLeft;

    Gtk::DrawingArea* sideViewTop;
    Gtk::DrawingArea* sideViewBottom;
};