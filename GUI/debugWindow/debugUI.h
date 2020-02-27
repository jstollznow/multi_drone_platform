#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/callback_queue.h>
#include <multi_drone_platform/droneLog.h>
#include "gtkRef.h"
#include "../../include/user_api.h"

#define UPDATE_RATE 10
#define UI_PATH "/home/jacob/catkin_ws/src/multi_drone_platform/GUI/debugWindow/debug.ui"

static std::map<int, std::string> logType = {
    {0, "INFO"}, {1, "WARN"}, {2, "DEBUG"},{3, "ERROR"}
};
/*
TYPE 
0 INFO
1 WARN
2 DEBUG
3 ERROR
*/


class debugUI: public Gtk::Window
{
    private:
        Glib::RefPtr<Gtk::Builder> builder;
        Gtk::Label* droneNameLabel;

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
        Gtk::Label* queueInput;
        Gtk::Label* batteryInput;
        Gtk::LevelBar* batteryLevelBar;
        Gtk::Label* speedMultiplierLabel;
        Gtk::Label* pktLossLabel;
        Gtk::TextView* logTextView;
        Gtk::TextBuffer* logTextBuffer;
                
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



    public:
        debugUI(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade);
        void init(mdp_api::id droneName, std::array<int, 2> startLocation = {0, 0}, bool expanded = false);
        void updateStats();

        // ros::CallbackQueue myQueue;
        // ros::AsyncSpinner mySpin;

    protected:
        mdp_api::id myDrone;
        std_msgs::Float32MultiArray updateMsg;
        
        ros::Subscriber logSubscriber;
        ros::NodeHandle myNode;
        
        
        bool expanded;
        bool first; 
        float firstTimeStamp;

        void linkWidgets();
        void linkWidget(std::string variableName, Gtk::Widget* widget);
        void on_landButton_clicked();
        void on_emergencyButton_clicked();
        void on_speedScale_value_changed();
        void on_expandButton_clicked();
        void on_debugWindow_destroy();
        void updateCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void logCallback(const multi_drone_platform::droneLog::ConstPtr& msg);
};