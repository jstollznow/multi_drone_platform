#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>

#include "multi_drone_platform/log.h"
#include "gtk_ref.h"
#include "user_api.h"

/**
 * How often the GUI is updated (in ms).
 */
#define UI_UPDATE_RATE 500

/**
 * Local location of Live Window xml file.
 */
#define UI_PATH "/src/debug/debug_app/debug.ui"

/**
 * Session directory ROS parameter name.
 */
#define SESSION_PARAM "/mdp/session_directory"

/**
 * Used to ease the repetitive code in the linking function (link_widgets())
 */
#define VNAME(x) #x

/**
 * Import error is thrown when a linkage error occurs between the xml file and the source cpp file.
 */
struct import_error : public std::exception {
    const char* what() const throw() {
        return "There was an import error, please check the xml file";
    }
};

class debug_window: public Gtk::Window {
public:
    /**
     * Asynchronous ROS event spinner used to periodically call queued callbacks
     */
    ros::AsyncSpinner windowSpinner;

    /**
     * Constructor for the Live View window. This begins by establishing the contents of the window through the
     * GtkBuilder type passed in. For each element that should be manipulated at anytime throughout the use of the
     * Live View window, they are linked to appropriate Gtk pointers. An import error is thrown if a linkage error
     * occurs.
     * @param cobject Generic type object passed to GtkWindow constructor.
     * @param refGlade Reference pointer to the GtkBuilder type associated with the appropriate ui file.
     */
    debug_window(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade);

    /**
     * This method is called to associate the window with a particular drone and display the window on the screen. It
     * links the Live View window to the drone's specific topics and links callback methods to GtkEvents. Prior to
     * diplay, the window is moved to the correct position as is passed in.
     * @param droneName The tag of the drone.
     * @param startLocation The x and y coordinate of the top corner pixel of the Live View window.
     * @param expanded Whether the window should be compressed or expanded.
     */
    void init(mdp::id droneName, std::array<int, 2> startLocation = {0, 0}, bool expanded = false);

    /**
     * This function is called periodically to update the GUI values according to the most recent updates from the
     * subscribed drone specific ROS topics. This is called periodically to avoid UI lock problems.
     */
    void update_ui_labels();
private:
    /**
     * Returns a reduced string based on the total number of seconds and the number of required decimal points, this
     * is used for logging purposes.
     * @param val ROS time total seconds (with decimals)
     * @param n Integer representing number precision of returned string
     * @return Returns a string associated with the ROS time value to be used in a log message.
     */
    std::string round_to_string(double val, int n);

    /**
     * Updates the state of the drone, into the currentState variable. If this variable does not exist the drone has
     * been removed and thus the window is closed.
     */
    void fetch_state_param();

    /**
     * Called periodically and will call all available queued callback events and update the state of the drone. A
     * dispatcher is then used to update the UI appropriately.
     * @return Always returns true.
     */
    bool ros_spin();

    /**
     * Called by the GtkDispatcher after the queued ROS callbacks had been called and updated the appropriate variables.
     * This function updated the UI elements, including the log and all the position/velocity current/desired labels.
     */
    void update_ui_on_resume();

    /**
     * Called from the constructor to associate all useful UI elements from the xml file with configurable Gtk pointers.
     */
    void link_widgets();

    /**
     * Event handler for the Land button click. Requests a land command for the drone.
     */
    void on_landButton_clicked();

    /**
     * Event handler for the Emergency button click. Requests an emergency command for the drone.
     */
    void on_emergencyButton_clicked();

    /**
     * Event handle for expand button click, will expand/compress window as required.
     */
    void on_expandButton_clicked();

    /**
     * Called during the window close event, exports all the text in the log text pane to a file and saves it to the
     * established session folder. The file is labelled according to the drone it is associated with.
     */
    void write_to_file();

    /**
     * High level function which firsts resets the safeguarding feedback panels, then iterating through each obstacles
     * position relative to the drones position applying the appropriate colours and direction on the feedback panels.
     * Closest (within the restricted distance) is red, then orange (within influence distance), then yellow (obstacle
     * that direction, but not effecting the drone).
     */
    void draw_obstacles();

    /**
     * Called when the window is closed, ensures the drone log messages are written to the session folder by calling
     * write_to_file.
     * @param event Gtk associated event type required for callback linkage.
     * @return
     */
    bool on_close(GdkEventAny* event);

    /**
     * Callback for the drone's obstacle ROS topic.
     * @param msg Vector of obstacle positions relative to the position of the drone.
     */
    void obstacle_callback(const geometry_msgs::PoseArray::ConstPtr& msg);

    /**
     * Callback for the drone's log ROS topic.
     * @param msg Contains log message.
     */
    void log_callback(const multi_drone_platform::log::ConstPtr& msg);

    /**
     * Callback for drone's current position ROS topic.
     * @param msg The drone's current position in the form of a PoseStamped msg.
     */
    void curr_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * Callback for the drone's desired position ROS topic.
     * @param msg The drone's desired position in the form of a PoseStamped.
     */
    void des_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    /**
     * Callback for the drone's current velocity ROS topic
     * @param msg The drone's current velocity in the form of a TwistStamped.
     */
    void curr_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    /**
     * Callback for the drone's desired velocity ROS topic
     * @param msg The drone's desired velocity as a TwistStamped.
     */
    void des_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    /**
     * Callback for the battery ROS topic
     * @param msg the battery percentage remaining for the drone, as a float.
     */
    void battery_callback(const std_msgs::Float32::ConstPtr& msg);

    /**
     * Fills the appropriate panel with the appropriate colour according to the level and direction of a particular
     * obstacle. Depending on which panel needs to be filled depends on which shape is drawn. The feedback panels
     * account for the length, width and height of the drone as specified in the relevant ROS parameters. For example,
     * if an obstacle is directly ahead of the drone it will appear in the top area of the top view panels, but may not
     * appear on the side view panels as it is directly inline rather than above or below (according to the height).
     * Where the top view panel is as follows -
     * 1 2 3
     * 4 X 5
     * 6 7 8
     * Side view panel is -
     * 1
     * X
     * 2
     */
    void fill_panel(geometry_msgs::Pose ob, int level);

    /**
     * Returns a vector of elements common between vector a and vector b. This is used in the process of deciding which
     * panels each obstacle should be drawn on.
     * @param a Vector representing Set A of panels
     * @param b Vector representing Set B of panels
     * @return Returns a vector of common numbers between the two vector inputs.
     */
    std::vector<int> reduce(std::vector<int> a, std::vector<int> b);

    /**
     * Uses the length of the drone to determine which panel an obstacle would appear relative to the drones position.
     * @param ob Position of the obstacle relative to the drone. Where the top view panel is as follows -
     * 1 2 3
     * 4 X 5
     * 6 7 8
     * @param candidates List of remaining panel options it could appear in.
     */
    void check_x(geometry_msgs::Pose ob, std::vector<int>& candidates);

    /**
     * Uses the width of the drone to determine which panel the obstacle should be displayed in accoridng to the drone's
     * width. Where the top view panel is as follows -
     * 1 2 3
     * 4 X 5
     * 6 7 8
     * @param ob Position of the obstacle relative to the drone.
     * @param candidates List of remaining panel options the obstacle could appear in.
     */
    void check_y(geometry_msgs::Pose ob, std::vector<int>& candidates);

    /**
     * Resets the color of all safeguarding panels to the default blue colour. Where the top view panel is as follows -
     * 1 2 3
     * 4 X 5
     * 6 7 8
     * Side view panel is -
     * 1
     * X
     * 2
     */
    void reset_all_panels();

    /**
     * Event called when the window should be drawn, involves drawing the window itself and also the safeguarding
     * feedback.
     * @param cr Gtk related input parameter, passed onto the appropriate Gtk event.
     * @return Returns the result of the Gtk::Window::on_draw event.
     */
    bool on_draw(const Cairo::RefPtr<Cairo::Context>& cr) override;

    /**
     * Callback queue to track callback events that have not been addressed.
     */
    ros::CallbackQueue windowQueue;

    /**
     * Stores the name and id of the drone with which the window is associated.
     */
    mdp::id myDrone;

    /**
     * A number of physical values obtained from ROS parameters associated with the drone.
     */
    double dWidth;
    double dHeight;
    double dLength;
    double dRestrictedDistance;
    double dInfluenceDistance;

    /**
     * ROS related types for topic subscrition and publishing.
     */
    ros::NodeHandle windowNode;
    ros::Subscriber logSubscriber;
    ros::Subscriber currTwistSubscriber;
    ros::Subscriber currPoseSubscriber;
    ros::Subscriber desPoseSubscriber;
    ros::Subscriber desTwistSubscriber;
    ros::Subscriber obstacleSubscriber;
    ros::Subscriber batterySubscriber;

    /**
     * ROS message types used to store the most recent updates which are accessed during the Gtk Dispatcher call.
     * Due to the use of the dispatcher, these variables are thread safe.
     */
    geometry_msgs::PoseStamped currPositionMsg;
    geometry_msgs::PoseStamped desPositionMsg;
    geometry_msgs::TwistStamped currVelocityMsg;
    geometry_msgs::TwistStamped desVelocityMsg;
    geometry_msgs::PoseArray obstacles;
    double batteryPercent;
    std::string currState;

    /**
     * Stores the max speed reached by the drone based on the current velocity at any given time.
     */
    double maxDroneSpeed;

    /**
     * Collects log updates in between GUI updates
     */
    std::string toAddToLog;

    /**
     * Whether the window is expanded or compressed
     */
    bool expanded;

    /**
     * Stores first time stamp recorded as the Live View window is started.
     */
    ros::Time firstTimeStamp;

    /**
     * Dispatcher which manages UI updates following ROS callback event updates.
     */
    Glib::Dispatcher dispatcher;

    /**
     * Unpacks the associated xml file and builds the window according the the layout defined in the xml file.
     */
    Glib::RefPtr<Gtk::Builder> builder;

    /**
     * Used to define the relative location and appearance of each stripe in the safeguarding feedback virtual
     * boundaries.
     */
    double inner = 0.3;
    double middle = 0.6;
    double outer = 0.9;
    double lineWidth = 0.1;

    /**
     * Default blue color used for the safeguarding feedback
     */
    std::array<double,3> defaultColor = std::array<double,3> {0.5, 0.8, 1.0};

    /**
     * Label for the drone name i.e. cflie_01
     */
    Gtk::Label* droneNameLabel;

    /**
     * Current Position labels
     */
    Gtk::Label* currPosX;
    Gtk::Label* currPosY;
    Gtk::Label* currPosZ;
    Gtk::Label* currYaw;

    /**
     * Current velocity labels
     */
    Gtk::Label* currVelX;
    Gtk::Label* currVelY;
    Gtk::Label* currVelZ;
    Gtk::Label* currYawRate;

    /**
     * Desired position labels
     */
    Gtk::Label* desPosX;
    Gtk::Label* desPosY;
    Gtk::Label* desPosZ;
    Gtk::Label* desYaw;

    /**
     * Desired velocity labels
     */
    Gtk::Label* desVelX;
    Gtk::Label* desVelY;
    Gtk::Label* desVelZ;
    Gtk::Label* desYawRate;

    /**
     * Extra information panel labels
     */
    Gtk::Label* stateInput;
    Gtk::LevelBar* batteryLevelBar;
    Gtk::Label* speedLabel;
    Gtk::Label* maxSpeedLabel;
    Gtk::Label* pktLossLabel;

    /**
     * Logging elements
     */
    Gtk::TextView* logTextView;
    Glib::RefPtr<Gtk::TextBuffer> logTextBuffer;
    Gtk::ScrolledWindow* logScroll;

    /**
     * User interaction elements
     */
    Gtk::Button* landButton;
    Gtk::Button* emergencyButton;
    Gtk::Button* expandButton;
    Gtk::Image* compressImage;
    Gtk::Image* expandImage;

    /**
     * Displayed/Hidden depending on whether the window is expanded or compressed.
     */
    Gtk::Grid* sidePanelGrid;

    /**
     * Drawing areas used for safeguarding feedback.
     */
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