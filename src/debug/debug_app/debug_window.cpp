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

    std::string logTopic = "mdp/drone_" + std::to_string(myDrone.numericID) + "/log";
    std::string velTopic = "mdp/drone_" + std::to_string(myDrone.numericID) + "/velocity";
    std::string posTopic = "mdp/drone_" + std::to_string(myDrone.numericID) + "/pose";
    windowNode.setCallbackQueue(&windowQueue);
    
    logSubscriber = windowNode.subscribe<multi_drone_platform::log>(logTopic, 10, &debug_window::log_callback, this);
    velSubscriber = windowNode.subscribe<geometry_msgs::TwistStamped>(velTopic, 1, &debug_window::velocity_callback, this);
    posSubscriber = windowNode.subscribe<geometry_msgs::PoseStamped>(posTopic, 1, &debug_window::position_callback, this);
    

}
void debug_window::init(mdp_api::id droneName, std::array<int, 2> startLocation, bool expanded) {
    this->expanded = false;
    if (expanded) on_expandButton_clicked();

    if (startLocation != std::array<int,2>({0, 0})) {
        this->move(startLocation[0], startLocation[1]);
    }

    myDrone = droneName;
    this->set_title(myDrone.name);
    droneNameLabel->set_label(myDrone.name);
    speedScale->set_round_digits(0);
    speedScale->set_value(5);
    speedMultiplierLabel->set_text("5");
    logTextBuffer->set_text("");
    
    this->show();
}

std::string debug_window::round_to_string(double val, int n) {
    std::ostringstream streamObj;
    streamObj << std::fixed << std::setprecision(n) << val;
    return streamObj.str();
}

void debug_window::update_stats() {
    currVelX->set_text(round_to_string(lastVelocityMsg.twist.linear.x, 3));
    currVelY->set_text(round_to_string(lastVelocityMsg.twist.linear.y, 3));
    currVelZ->set_text(round_to_string(lastVelocityMsg.twist.linear.z, 3));
    currYawRate->set_text(round_to_string(lastVelocityMsg.twist.angular.z, 3));

    currPosX->set_text(round_to_string(lastPositionMsg.pose.position.x, 3));
    currPosY->set_text(round_to_string(lastPositionMsg.pose.position.y, 3));
    currPosZ->set_text(round_to_string(lastPositionMsg.pose.position.z, 3));
    // currYaw->set_text(round_to_string(lastPositionMsg.orient.angular.z, 5));
}
void debug_window::update_ros_widgets() {
    logTextBuffer->insert(logTextBuffer->end(), toAddToLog);
    toAddToLog = "";
    Glib::RefPtr<Gtk::Adjustment> scrollAdjust = logScroll->get_vadjustment();
    scrollAdjust->set_value(scrollAdjust->get_upper());
    update_stats();
}
bool debug_window::ros_spin() {
    windowQueue.callAvailable();
    dispatcher.emit();
    return true;
}

void debug_window::on_landButton_clicked() {
    
}
void debug_window::on_emergencyButton_clicked() {

}

void debug_window::log_callback(const multi_drone_platform::log::ConstPtr& msg) {
    if (first) {
        first = false;
        firstTimeStamp = msg->timeStamp;
    }

    // fix time stamp
    double time = ros::Duration(msg->timeStamp - firstTimeStamp).toSec();

    std::string newLogLine = round_to_string(time, 4) + ": " + msg->type + " " + msg->logMessage + "\n";

    toAddToLog += newLogLine;
}

void debug_window::velocity_callback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    lastVelocityMsg = *(msg.get());
    dispatcher.emit();
    // queue update
}
void debug_window::position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    lastPositionMsg = *(msg.get());
    // queue update
}

void debug_window::on_speedScale_value_changed() {
    std::ostringstream streamObj;
	streamObj << std::fixed;
	streamObj << std::setprecision(0);
	streamObj << speedScale->get_value();
  
    speedMultiplierLabel->set_label(streamObj.str());
}
void debug_window::on_expandButton_clicked() {
    sidePanelGrid->set_visible(!expanded);
    logScroll->set_visible(!expanded);
    expandButton->set_image(expanded ? *expandImage : *compressImage);
    this->resize(1, 1);
    expanded = !expanded;
}


void debug_window::on_debugWindow_destroy() {
    std::cout<<"TCHUSSSS"<<std::endl;
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
        builder->get_widget(VNAME(queueInput), queueInput);
        builder->get_widget(VNAME(batteryInput), batteryInput);
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
        builder->get_widget(VNAME(droneStatTable), droneStatTable);
        builder->get_widget(VNAME(topViewBottom), topViewBottom);
        builder->get_widget(VNAME(topViewTop), topViewTop);
        builder->get_widget(VNAME(topViewLeft), topViewLeft);
        builder->get_widget(VNAME(topViewRight), topViewRight);
        builder->get_widget(VNAME(topViewTopLeft), topViewTopLeft);
        builder->get_widget(VNAME(topViewTopRight), topViewTopRight);
        builder->get_widget(VNAME(topViewBotRight), topViewBotRight);
        builder->get_widget(VNAME(topViewBotLeft), topViewBotLeft);
        builder->get_widget(VNAME(sideViewTop), sideViewTop);
        builder->get_widget(VNAME(sideViewBottom), sideViewBottom);
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
        

        Glib::signal_timeout().connect( sigc::mem_fun(*this, &debug_window::ros_spin), LOG_POST_RATE);

        // Glib::signal_idle().connect(sigc::mem_fun(*this, &debug_window::ros_spin) );

        dispatcher.connect(sigc::mem_fun(*this, &debug_window::update_ros_widgets));

    }
    catch(const std::exception& e) {
        throw importError();
    }   
}
