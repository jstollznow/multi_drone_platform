#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <exception>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/callback_queue.h>
#include "multi_drone_platform/log.h"
#include "gtk_ref.h"
#include "user_api.h"

#define UPDATE_RATE 10
#define LOG_POST_RATE 3000
#define UI_PATH "/home/jacob/catkin_ws/src/multi_drone_platform/src/debug/debug_app/debug.ui"

class debug_window: public Gtk::Window {
    private:
        Glib::RefPtr<Gtk::Builder> builder;
        Gtk::Label* droneNameLabel;
        std::string logTopic;

        // for status updates
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

        Gtk::Label* stateInput;
        Gtk::LevelBar* batteryLevelBar;
        Gtk::Label* speedMultiplierLabel;
        Gtk::Label* pktLossLabel;
        Gtk::TextView* logTextView;
        Glib::RefPtr<Gtk::TextBuffer> logTextBuffer;

                
        // for user interaction
        Gtk::Button* landButton;
        Gtk::Button* emergencyButton;
        Gtk::Scale* speedScale;
        Gtk::Button* expandButton;

        Gtk::Grid* sidePanelGrid;
        Gtk::Grid* droneStatTable;

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
        Gtk::ScrolledWindow* logScroll;

        Gtk::Image* compressImage;
        Gtk::Image* expandImage;   

        std::string round_to_string(double val, int n);

    public:
        debug_window(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade);
        void init(mdp_api::id droneName, std::array<int, 2> startLocation = {0, 0}, bool expanded = false);
        void update_stats();
        ros::AsyncSpinner windowSpinner;        
        ros::CallbackQueue windowQueue;

    protected:
        multi_drone_platform::log mostRecentLog;
        mdp_api::id myDrone;

        Glib::Dispatcher dispatcher;

        ros::NodeHandle windowNode;
        ros::Subscriber logSubscriber;
        ros::Subscriber currTwistSubscriber;
        ros::Subscriber currPoseSubscriber;
        ros::Subscriber desPoseSubscriber;
        ros::Subscriber desTwistSubscriber;

        std::string currState;

        geometry_msgs::PoseStamped currPositionMsg;
        geometry_msgs::PoseStamped desPositionMsg;

        geometry_msgs::TwistStamped currVelocityMsg;
        geometry_msgs::TwistStamped desVelocityMsg;



        std::string toAddToLog;

        bool expanded;
        bool first; 
        ros::Time firstTimeStamp;
        bool ros_spin();
        void update_ros_widgets();
        void link_widgets();
        void link_widget(std::string variableName, Gtk::Widget* widget);
        void on_landButton_clicked();
        void on_emergencyButton_clicked();
        void on_speedScale_value_changed();
        void on_expandButton_clicked();
        void on_debugWindow_destroy();
        
        void log_callback(const multi_drone_platform::log::ConstPtr& msg);
        void curr_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void des_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void curr_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
        void des_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
};