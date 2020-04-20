#include "debug_window.h"

#define VNAME(x) #x

struct importError : public std::exception {
    const char* what() const throw() {
        return "There was an import error, please check the xml file";
    }
};

debug_window::debug_window(BaseObjectType* cobject, const Glib::RefPtr<Gtk::Builder>& refGlade): 
Gtk::Window(cobject), builder(refGlade), windowSpinner(1,&windowQueue), dispatcher() {
    try {
        link_widgets();
    }
    catch(importError &e) {
        std::cout<<e.what()<<std::endl;
    }
    first = true;
    windowNode = ros::NodeHandle();
    
    toAddToLog = "";
    

}
void debug_window::init(mdp::id droneName, std::array<int, 2> startLocation, bool expanded) {

    myDrone = droneName;

    logTopic = "mdp/drone_" + std::to_string(myDrone.numericID) + "/log";

    std::string currPoseTopic = "mdp/drone_" + std::to_string(myDrone.numericID) + "/curr_pose";
    std::string desPoseTopic = "mdp/drone_" + std::to_string(myDrone.numericID) + "/des_pose";

    std::string currTwistTopic = "mdp/drone_" + std::to_string(myDrone.numericID) + "/curr_twist";
    std::string desTwistTopic = "mdp/drone_" + std::to_string(myDrone.numericID) + "/des_twist";

    windowNode.setCallbackQueue(&windowQueue);
    
    logSubscriber = windowNode.subscribe<multi_drone_platform::log>(
            logTopic,
            10,
            &debug_window::log_callback,
            this);

    currPoseSubscriber = windowNode.subscribe<geometry_msgs::PoseStamped>(
            currPoseTopic,
            1,
            &debug_window::curr_position_callback,
            this);
    desPoseSubscriber = windowNode.subscribe<geometry_msgs::PoseStamped>(
            currPoseTopic,
            1,
            &debug_window::des_position_callback,
            this);

    currTwistSubscriber = windowNode.subscribe<geometry_msgs::TwistStamped>(
            currTwistTopic,
            1,
            &debug_window::curr_velocity_callback,
            this);
    desTwistSubscriber = windowNode.subscribe<geometry_msgs::TwistStamped>(
            desTwistTopic,
            1,
            &debug_window::des_velocity_callback,
            this);

    this->expanded = false;
    this->set_title(myDrone.name);

    droneNameLabel->set_label(myDrone.name);
    speedScale->set_round_digits(0);
    speedScale->set_value(5);
    speedMultiplierLabel->set_text("5");
    logTextBuffer->set_text("");

    if (expanded) on_expandButton_clicked();

    if (startLocation != std::array<int,2>({0, 0})) {
        this->move(startLocation[0], startLocation[1]);
    }


    this->show();
}

std::string debug_window::round_to_string(double val, int n) {
    std::ostringstream streamObj;
    streamObj << std::fixed << std::setprecision(n) << val;
    return streamObj.str();
}

void debug_window::update_ui_labels() {
    int decimals = 3;

    currVelX->set_text(round_to_string(currVelocityMsg.twist.linear.x, decimals));
    currVelY->set_text(round_to_string(currVelocityMsg.twist.linear.y, decimals));
    currVelZ->set_text(round_to_string(currVelocityMsg.twist.linear.z, decimals));
    currYawRate->set_text(round_to_string(currVelocityMsg.twist.angular.z, decimals));

    currPosX->set_text(round_to_string(currPositionMsg.pose.position.x, decimals));
    currPosY->set_text(round_to_string(currPositionMsg.pose.position.y, decimals));
    currPosZ->set_text(round_to_string(currPositionMsg.pose.position.z, decimals));
    // @TODO: fix yaw system wide
    // currYaw->set_text(round_to_string(currPositionMsg.orient.angular.z, 5));


    desVelX->set_text(round_to_string(desVelocityMsg.twist.linear.x, decimals));
    desVelY->set_text(round_to_string(desVelocityMsg.twist.linear.y, decimals));
    desVelZ->set_text(round_to_string(desVelocityMsg.twist.linear.z, decimals));
    desYawRate->set_text(round_to_string(desVelocityMsg.twist.angular.z, decimals));

    desPosX->set_text(round_to_string(desPositionMsg.pose.position.x, decimals));
    desPosY->set_text(round_to_string(desPositionMsg.pose.position.y, decimals));
    desPosZ->set_text(round_to_string(desPositionMsg.pose.position.z, decimals));
    // currYaw->set_text(round_to_string(currPositionMsg.orient.angular.z, 5));

    stateInput->set_text(currState);
}
void debug_window::update_ui_on_resume() {
    logTextBuffer->insert(logTextBuffer->end(), toAddToLog);

    toAddToLog = "";
    update_ui_labels();

//    Glib::RefPtr<Gtk::Adjustment> scrollAdjust = logScroll->get_vadjustment();
//    scrollAdjust->set_value(scrollAdjust->get_upper());

}
void debug_window::fetch_state_param() {
    if (windowNode.hasParam("/mdp/drone_" + std::to_string(myDrone.numericID) + "/state")) {
        windowNode.getParam("/mdp/drone_" + std::to_string(myDrone.numericID) + "/state", currState);
    }
}
bool debug_window::ros_spin() {
    windowQueue.callAvailable();
    fetch_state_param();

    dispatcher.emit();
    return true;
}

void debug_window::on_landButton_clicked() {
    mdp::cmd_land(myDrone);
}
void debug_window::on_emergencyButton_clicked() {
    mdp::cmd_emergency(myDrone);
}

void debug_window::log_callback(const multi_drone_platform::log::ConstPtr& msg) {
    if (first) {
        first = false;
        firstTimeStamp = msg->timeStamp;
    }

    double time = ros::Duration(msg->timeStamp - firstTimeStamp).toSec();

    std::string newLogLine = round_to_string(time, 4) + ": " + msg->type + " " + msg->logMessage + "\n";

    toAddToLog += newLogLine;
}

void debug_window::curr_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    currVelocityMsg = *(msg.get());
}
void debug_window::curr_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    currPositionMsg = *(msg.get());
}
void debug_window::des_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    desPositionMsg = *(msg.get());
}
void debug_window::des_velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    desVelocityMsg = *(msg.get());
}
void debug_window::on_speedScale_value_changed() {
    std::ostringstream streamObj;
	streamObj << std::fixed;
	streamObj << std::setprecision(0);
	streamObj << speedScale->get_value();
  
    speedMultiplierLabel->set_label(streamObj.str());
}
void debug_window::on_expandButton_clicked() {
    if (!expanded) {
        logSubscriber = windowNode.subscribe<multi_drone_platform::log>(logTopic, 10, &debug_window::log_callback, this);
    }
    else {
        logSubscriber.shutdown();
    }
    sidePanelGrid->set_visible(!expanded);
    logScroll->set_visible(!expanded);
    expandButton->set_image(expanded ? *expandImage : *compressImage);
    this->resize(1, 1);
    expanded = !expanded;
}


void debug_window::on_debugWindow_destroy() {
    std::cout<<"TCHUSSSS"<<std::endl;
}

void debug_window::on_logTextBuffer_changed() {

}


void debug_window::link_widgets() {
    try {
        builder->get_widget(VNAME(droneNameLabel), droneNameLabel);
        builder->get_widget(VNAME(droneNameLabel), droneNameLabel);
        builder->get_widget(VNAME(currPosX), currPosX);
        builder->get_widget(VNAME(currPosY), currPosY);
        builder->get_widget(VNAME(currPosZ), currPosZ);
        builder->get_widget(VNAME(currYaw), currYaw);
        builder->get_widget(VNAME(currVelX), currVelX);
        builder->get_widget(VNAME(currVelY), currVelY);
        builder->get_widget(VNAME(currVelZ), currVelZ);
        builder->get_widget(VNAME(currYawRate), currYawRate);
        builder->get_widget(VNAME(desPosX), desPosX);
        builder->get_widget(VNAME(desPosY), desPosY);
        builder->get_widget(VNAME(desPosZ), desPosZ);
        builder->get_widget(VNAME(desYaw), desYaw);
        builder->get_widget(VNAME(desVelX), desVelX);
        builder->get_widget(VNAME(desVelY), desVelY);
        builder->get_widget(VNAME(desVelZ), desVelZ);
        builder->get_widget(VNAME(desYawRate), desYawRate);
        builder->get_widget(VNAME(stateInput), stateInput);
        builder->get_widget(VNAME(batteryLevelBar), batteryLevelBar);
        builder->get_widget(VNAME(speedMultiplierLabel), speedMultiplierLabel);
        builder->get_widget(VNAME(pktLossLabel), pktLossLabel);
        builder->get_widget(VNAME(logTextView), logTextView);
        logTextBuffer = logTextView->get_buffer();
        builder->get_widget(VNAME(landButton), landButton);
        builder->get_widget(VNAME(emergencyButton), emergencyButton);
        builder->get_widget(VNAME(speedScale), speedScale);
        builder->get_widget(VNAME(expandButton), expandButton);
        builder->get_widget(VNAME(sidePanelGrid), sidePanelGrid);
        builder->get_widget(VNAME(topViewBottom), topViewBottom);
        builder->get_widget(VNAME(topViewTop), topViewTop);
        builder->get_widget(VNAME(topViewLeft), topViewLeft);
        builder->get_widget(VNAME(topViewRight), topViewRight);
        builder->get_widget(VNAME(topViewTopLeft), topViewTopLeft);
        builder->get_widget(VNAME(topViewTopRight), topViewTopRight);
        builder->get_widget(VNAME(topViewBotRight), topViewBotRight);
        builder->get_widget(VNAME(topViewBotLeft), topViewBotLeft);
        builder->get_widget(VNAME(logScroll),logScroll);
        builder->get_widget(VNAME(compressImage), compressImage);
        builder->get_widget(VNAME(expandImage), expandImage);

        landButton->signal_clicked().connect
        (sigc::mem_fun(*this, &debug_window::on_landButton_clicked));
        
        emergencyButton->signal_clicked().connect
        (sigc::mem_fun(*this, &debug_window::on_emergencyButton_clicked));
        
        speedScale->signal_value_changed().connect
        (sigc::mem_fun(*this, &debug_window::on_speedScale_value_changed));

        expandButton->signal_clicked().connect
        (sigc::mem_fun(*this, &debug_window::on_expandButton_clicked));
        
        logTextBuffer->signal_changed().connect(
                sigc::mem_fun(*this, &debug_window::on_logTextBuffer_changed)
                );

        Glib::signal_timeout().connect( sigc::mem_fun(*this, &debug_window::ros_spin), LOG_POST_RATE);

        dispatcher.connect(sigc::mem_fun(*this, &debug_window::update_ui_on_resume));

    }
    catch(const std::exception& e) {
        throw importError();
    }   
}
