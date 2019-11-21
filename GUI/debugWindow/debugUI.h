#include "gtkRef.h"
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <ros/ros.h>
#include "../../include/user_api.h"
// #include <ros/ros.h>
// #include "../../src/drone_server/drone_server.h"

#define UI_PATH "/home/jacob/catkin_ws/src/multi_drone_platform/GUI/debugWindow/debug.ui"

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

        mdp_api::id myDrone;

        // ros::NodeHandle myNode;
        ros::NodeHandle myNode;

        bool expanded;
    public:
        debugUI(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade);
        void init(mdp_api::id droneName, std::array<int, 2> startLocation = {0, 0}, bool expanded = false);
        void updateStats();

    protected:
        void linkWidgets();
        void linkWidget(std::string variableName, Gtk::Widget* widget);
        void on_landButton_clicked();
        void on_emergencyButton_clicked();
        void on_speedScale_value_changed();
        void on_expandButton_clicked();
        void on_debugWindow_destroy();
};