#include "debug_window.h"
#include <fstream>
#include <ros/package.h>
#include <math.h>
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
    maxMag = 0.0f;
    myDrone = droneName;

    windowNode.getParam("mdp/drone_" + std::to_string(myDrone.numericID) + "/width", this->dWidth);
    windowNode.getParam("mdp/drone_" + std::to_string(myDrone.numericID) + "/height", this->dHeight);
    windowNode.getParam("mdp/drone_" + std::to_string(myDrone.numericID) + "/length", this->dLength);
    windowNode.getParam("mdp/drone_" + std::to_string(myDrone.numericID) + "/restrictedDistance", this->dRestrictedDistance);
    windowNode.getParam("mdp/drone_" + std::to_string(myDrone.numericID) + "/influenceDistance", this->dInfluenceDistance);

    droneSpeedMultiplier = 1.0;
    logTopic = "mdp/drone_" + std::to_string(myDrone.numericID) + "/log";
    firstTimeStamp = ros::Time().now();
    logTextBuffer->set_text(
            round_to_string(firstTimeStamp.toSec(), 4) +
            ": All timing relative to " +
            std::to_string(firstTimeStamp.toSec()) +
            "\n");
    std::string currPoseTopic = "mdp/drone_" + std::to_string(myDrone.numericID) + "/curr_pose";
    std::string desPoseTopic = "mdp/drone_" + std::to_string(myDrone.numericID) + "/des_pose";

    std::string currTwistTopic = "mdp/drone_" + std::to_string(myDrone.numericID) + "/curr_twist";
    std::string desTwistTopic = "mdp/drone_" + std::to_string(myDrone.numericID) + "/des_twist";

    std::string obstacleTopic = "mdp/drone_" + std::to_string(myDrone.numericID) + "/obstacles";

    windowNode.setCallbackQueue(&windowQueue);
    
    logSubscriber = windowNode.subscribe<multi_drone_platform::log>(
            logTopic,
            50,
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
    obstacleSubscriber = windowNode.subscribe<geometry_msgs::PoseArray>(
            obstacleTopic,
            1,
            &debug_window::obstacle_callback,
            this);


    this->expanded = false;
    this->set_title(myDrone.name);

    droneNameLabel->set_label(myDrone.name);
    speedScale->set_round_digits(0);
    speedScale->set_value(5);
    speedMultiplierLabel->set_text("1.0x");

    if (expanded) on_expandButton_clicked();

    if (startLocation != std::array<int,2>({0, 0})) {
        this->move(startLocation[0], startLocation[1]);
    }
    this->show();

}

std::string debug_window::round_to_string(double val, int n) {
    double deci = val - (int)val;
    val = (int)val % 10000;
    val += deci;
    std::ostringstream streamObj;
    streamObj << std::fixed << std::setprecision(n) << val;
    return streamObj.str();
}

void debug_window::fill_panel(geometry_msgs::Pose ob, int level, std::array<double, 3> color) {
    std::vector<int> panels(8);
    std::iota (std::begin(panels), std::end(panels), 1);

    check_x(ob, panels);
    check_y(ob, panels);

    int width = 0;
    int height = 0;
    Gtk::Allocation allocation;
    Cairo::RefPtr<Cairo::Context> cr;

    if (panels.size() == 1) {
        switch (panels[0]) {
            case 1:
                cr = topViewTopLeft->get_window()->create_cairo_context();
                allocation = topViewTopLeft->get_allocation();
                cr->scale(allocation.get_width(), allocation.get_height());
                switch (level) {
                    case 3:
                        cr->move_to(1.0 - outer,1.0);
                        cr->arc(1.0, 1.0, outer, M_PI, M_PI/2);
                    case 2:
                        cr->move_to(1.0 - middle,1.0);
                        cr->arc(1.0, 1.0, middle, M_PI, M_PI/2);
                    case 1:
                        cr->move_to(1.0 - inner,1.0);
                        cr->arc(1.0, 1.0, inner, M_PI, M_PI/2);
                }
                break;
            case 2:
                cr = topViewTop->get_window()->create_cairo_context();
                allocation = topViewTop->get_allocation();
                cr->scale(allocation.get_width(), allocation.get_height());
                switch(level) {
                    case 3:
                        cr->move_to(0.0,1.0 - outer);
                        cr->line_to(1.0, 1.0 - outer);
                    case 2:
                        cr->move_to(0.0,1.0 - middle);
                        cr->line_to(1.0, 1.0 - middle);
                    case 1:
                        cr->move_to(0.0,1.0 - inner);
                        cr->line_to(1.0, 1.0 - inner);
                }
                break;
            case 3:
                cr = topViewTopRight->get_window()->create_cairo_context();
                allocation = topViewTopRight->get_allocation();
                cr->scale(allocation.get_width(), allocation.get_height());
                switch(level) {
                    case 3:
                        cr->arc(0.0, 1.0, outer, M_PI/2, 0.0);
                    case 2:
                        cr->arc(0.0, 1.0, middle, M_PI/2, 0.0);
                    case 1:
                        cr->arc(0.0, 1.0, inner, M_PI/2, 0.0);
                }
                break;
            case 4:
                cr = topViewLeft->get_window()->create_cairo_context();
                allocation = topViewLeft->get_allocation();
                cr->scale(allocation.get_width(), allocation.get_height());
                switch(level) {
                    case 3:
                        cr->move_to(1.0 - outer, 0.0);
                        cr->line_to(1.0 - outer, 1.0);
                    case 2:
                        cr->move_to(1.0 - middle, 0.0);
                        cr->line_to(1.0 - middle, 1.0);
                    case 1:
                        cr->move_to(1.0-inner,0.0);
                        cr->line_to(1.0-inner, 1.0);
                }
                break;
            case 5:
                cr = topViewRight->get_window()->create_cairo_context();
                allocation = topViewRight->get_allocation();
                cr->scale(allocation.get_width(), allocation.get_height());
                switch(level) {
                    case 3:
                        cr->move_to(outer, 0.0);
                        cr->line_to(outer, 1.0);
                    case 2:
                        cr->move_to(middle, 0.0);
                        cr->line_to(middle, 1.0);
                    case 1:
                        cr->move_to(inner,0.0);
                        cr->line_to(inner, 1.0);
                }
                break;
            case 6:
                cr = topViewBotLeft->get_window()->create_cairo_context();
                allocation = topViewBotLeft->get_allocation();
                cr->scale(allocation.get_width(), allocation.get_height());
                switch(level) {
                    case 3:
                        cr->move_to(1.0-outer,0.0);
                        cr->arc(1.0, 0.0, outer, -M_PI/2, -M_PI);
                    case 2:
                        cr->move_to(1.0-middle,0.0);
                        cr->arc(1.0, 0.0, middle, -M_PI/2, -M_PI);
                    case 1:
                        cr->move_to(1.0-inner, 0.0);
                        cr->arc(1.0, 0.0, inner, -M_PI/2, -M_PI);
                }
                break;
            case 7:
                cr = topViewBottom->get_window()->create_cairo_context();
                allocation = topViewBottom->get_allocation();
                cr->scale(allocation.get_width(), allocation.get_height());
                switch(level) {
                    case 3:
                        cr->move_to(0.0,outer);
                        cr->line_to(1.0, outer);
                    case 2:
                        cr->move_to(0.0,middle);
                        cr->line_to(1.0, middle);
                    case 1:
                        cr->move_to(0.0,inner);
                        cr->line_to(1.0, inner);
                }
                break;
            case 8:
                cr = topViewBotRight->get_window()->create_cairo_context();
                allocation = topViewBotRight->get_allocation();
                cr->scale(allocation.get_width(), allocation.get_height());
                switch(level) {
                    case 3:
                        cr->move_to(outer,0.0);
                        cr->arc(0.0, 0.0, outer, 0, -M_PI/2);
                    case 2:
                        cr->move_to(middle,0.0);
                        cr->arc(0.0, 0.0, middle, 0, -M_PI/2);
                    case 1:
                        cr->move_to(inner, 0.0);
                        cr->arc(0.0, 0.0, inner, 0, -M_PI/2);
                }
                break;
        }
        cr->set_source_rgb(color[0], color[1], color[2]);
        cr->set_line_width(lineWidth);
        cr->stroke();
    }

}

bool debug_window::on_draw(const Cairo::RefPtr<Cairo::Context>& cr) {
    draw_obstacles();
    return Gtk::Window::on_draw(cr);
}

std::vector<int> debug_window::reduce(std::vector<int> a, std::vector<int> b) {
    std::vector<int> ans;
    for (auto valid : a) {
        if (std::find(b.begin(), b.end(), valid) != b.end()) {
            ans.push_back(valid);
        }
    }
    return ans;
}

void debug_window::check_x(geometry_msgs::Pose ob, std::vector<int> &candidates) {
    if (ob.position.x >= dLength/2) {
        candidates = reduce(std::vector<int>{1,2,3}, candidates);
    }
    else if (ob.position.x <= -dLength/2) {
        candidates = reduce(std::vector<int>{6,7,8}, candidates);
    }
    else {
        candidates = reduce(std::vector<int>{4,5}, candidates);
    }
}

void debug_window::check_y(geometry_msgs::Pose ob, std::vector<int> &candidates) {
    if (ob.position.y >= dWidth/2) {
        candidates = reduce(std::vector<int>{3,5,8}, candidates);
    }
    else if (ob.position.y <= -dWidth/2) {
        candidates = reduce(std::vector<int>{1,4,6}, candidates);
    }
    else {
        candidates = reduce(std::vector<int>{2,7}, candidates);
    }
}

void debug_window::draw_obstacles() {
    reset_all_panels();
    for (int i = obstacles.poses.size() - 1; i >= 0; i --) {
        geometry_msgs::Pose ob = obstacles.poses[i];
        int level = 0;
        std::array<double, 3> highlightColor {0.0, 0.0, 0.0};
        // distance
        if (ob.orientation.x <= dRestrictedDistance) {
            // red
            highlightColor = {0.8, 0.0, 0.10};
            level = 3;
        }
        else if (ob.orientation.x <= dInfluenceDistance) {
            // orange
            highlightColor = {0.9, 0.50, 0.00};
            level = 2;
        }
        else {
            // yellow
            highlightColor = {1.0, 0.90, 0.00};
            level = 1;
        }
        fill_panel(ob, level, highlightColor);
    }

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
     currYaw->set_text(round_to_string(currPositionMsg.pose.orientation.z, 5));

    desVelX->set_text(round_to_string(desVelocityMsg.twist.linear.x, decimals));
    desVelY->set_text(round_to_string(desVelocityMsg.twist.linear.y, decimals));
    desVelZ->set_text(round_to_string(desVelocityMsg.twist.linear.z, decimals));
    desYawRate->set_text(round_to_string(desVelocityMsg.twist.angular.z, decimals));

    desPosX->set_text(round_to_string(desPositionMsg.pose.position.x, decimals));
    desPosY->set_text(round_to_string(desPositionMsg.pose.position.y, decimals));
    desPosZ->set_text(round_to_string(desPositionMsg.pose.position.z, decimals));
    desYaw->set_text(round_to_string(desPositionMsg.pose.orientation.z, 5));
    double vel = std::sqrt(
            currVelocityMsg.twist.linear.x * currVelocityMsg.twist.linear.x +
            currVelocityMsg.twist.linear.y * currVelocityMsg.twist.linear.y +
            currVelocityMsg.twist.linear.z + currVelocityMsg.twist.linear.z
            );
    maxMag = std::max(maxMag, vel);
    currYaw->set_text(round_to_string(maxMag, decimals));

    stateInput->set_text(currState);

    draw_obstacles();
}
void debug_window::update_ui_on_resume() {
    logTextBuffer->insert(logTextBuffer->end(), toAddToLog);
    toAddToLog = "";
    update_ui_labels();
//    Glib::RefPtr<Gtk::Adjustment> scrollAdjust = logScroll->get_vadjustment();
//    scrollAdjust->set_value(scrollAdjust->get_upper());
}

void debug_window::write_to_file() {
    std::ofstream file;
    std::string session = "";
    if (ros::param::has(SESSION_PARAM)) {
        ros::param::get(SESSION_PARAM, session);
    }
    std::string fileName = "drone_" + std::to_string(myDrone.numericID) + ".txt";
    file.open(session + fileName);
    file << logTextBuffer->get_text(true);
    file.close();
}
void debug_window::fetch_state_param() {
    if (windowNode.hasParam("/mdp/drone_" + std::to_string(myDrone.numericID) + "/state")) {
        windowNode.getParam("/mdp/drone_" + std::to_string(myDrone.numericID) + "/state", currState);
    }
    else {
        this->close();
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
    double time = ros::Duration(msg->timeStamp - firstTimeStamp).toSec();
    std::string newLogLine = round_to_string(msg->timeStamp.toSec(), 4) + ": " + msg->type + " " + msg->logMessage + "\n";
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
void debug_window::obstacle_callback(const geometry_msgs::PoseArray::ConstPtr &msg) {
    obstacles = *(msg.get());
}
void debug_window::on_speedScale_value_changed() {
    if ((int)speedScale->get_value()<= 5) {
        droneSpeedMultiplier = speedScale->get_value() * 0.2;
    }
    else {
        droneSpeedMultiplier = 2.0;
    }

    const std::string multi = round_to_string(droneSpeedMultiplier, 1) + "x";
    speedMultiplierLabel->set_text(multi);
}
void debug_window::on_expandButton_clicked() {
    if (!expanded) {
        logSubscriber = windowNode.subscribe<multi_drone_platform::log>(logTopic, 50, &debug_window::log_callback, this);
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

bool debug_window::on_close(GdkEventAny* event) {
    write_to_file();
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
        builder->get_widget(VNAME(sideViewTop), sideViewTop);
        builder->get_widget(VNAME(sideViewBottom),sideViewBottom);
        builder->get_widget(VNAME(logScroll), logScroll);
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

        Glib::signal_timeout().connect(sigc::mem_fun(*this, &debug_window::ros_spin), LOG_POST_RATE);
        this->signal_delete_event().connect(
                sigc::mem_fun(*this, &debug_window::on_close)
        );
        dispatcher.connect(sigc::mem_fun(*this, &debug_window::update_ui_on_resume));
    }
    catch (const std::exception &e) {
        throw importError();
    }
}

void debug_window::reset_all_panels() {
    Cairo::RefPtr<Cairo::Context> cr = topViewTopLeft->get_window()->create_cairo_context();
    Gtk::Allocation allocation = topViewTopLeft->get_allocation();
    const int width = allocation.get_width();
    const int height = allocation.get_height();
    cr->set_line_width(lineWidth);
    cr->set_source_rgb(defaultColor[0], defaultColor[1], defaultColor[2]);
    cr->scale(width, height);
    cr->move_to(1.0 - inner,1.0);
    cr->arc(1.0, 1.0, inner, M_PI, M_PI/2);
    cr->move_to(1.0 - middle,1.0);
    cr->arc(1.0, 1.0, middle, M_PI, M_PI/2);
    cr->move_to(1.0 - outer,1.0);
    cr->arc(1.0, 1.0, outer, M_PI, M_PI/2);
    cr->stroke();

    cr = topViewTop->get_window()->create_cairo_context();
    cr->set_line_width(lineWidth);
    cr->set_source_rgb(defaultColor[0], defaultColor[1], defaultColor[2]);
    cr->scale(width, height);
    cr->move_to(0.0,1.0 - outer);
    cr->line_to(1.0, 1.0 - outer);
    cr->move_to(0.0,1.0 - middle);
    cr->line_to(1.0, 1.0 - middle);
    cr->move_to(0.0,1.0 - inner);
    cr->line_to(1.0, 1.0 - inner);
    cr->stroke();

    cr = topViewTopRight->get_window()->create_cairo_context();
    cr->set_line_width(lineWidth);
    cr->set_source_rgb(defaultColor[0], defaultColor[1], defaultColor[2]);
    cr->scale(width, height);
    cr->arc(0.0, 1.0, inner, M_PI/2, 0.0);
    cr->arc(0.0, 1.0, middle, M_PI/2, 0.0);
    cr->arc(0.0, 1.0, outer, M_PI/2, 0.0);
    cr->stroke();

    cr = topViewLeft->get_window()->create_cairo_context();
    cr->set_line_width(lineWidth);
    cr->set_source_rgb(defaultColor[0], defaultColor[1], defaultColor[2]);
    cr->scale(width, height);
    cr->move_to(1.0-inner,0.0);
    cr->line_to(1.0-inner, 1.0);
    cr->move_to(1.0 - middle, 0.0);
    cr->line_to(1.0 - middle, 1.0);
    cr->move_to(1.0 - outer, 0.0);
    cr->line_to(1.0 - outer, 1.0);
    cr->stroke();

    cr = topViewRight->get_window()->create_cairo_context();
    cr->set_line_width(lineWidth);
    cr->set_source_rgb(defaultColor[0], defaultColor[1], defaultColor[2]);
    cr->scale(width, height);
    cr->move_to(inner,0.0);
    cr->line_to(inner, 1.0);
    cr->move_to(middle, 0.0);
    cr->line_to(middle, 1.0);
    cr->move_to(outer, 0.0);
    cr->line_to(outer, 1.0);
    cr->stroke();

    cr = topViewBotLeft->get_window()->create_cairo_context();
    cr->set_line_width(lineWidth);
    cr->set_source_rgb(defaultColor[0], defaultColor[1], defaultColor[2]);
    cr->scale(width, height);
    cr->move_to(1.0-inner, 0.0);
    cr->arc(1.0, 0.0, inner, -M_PI/2, -M_PI);
    cr->move_to(1.0-middle,0.0);
    cr->arc(1.0, 0.0, middle, -M_PI/2, -M_PI);
    cr->move_to(1.0-outer,0.0);
    cr->arc(1.0, 0.0, outer, -M_PI/2, -M_PI);
    cr->stroke();

    cr = topViewBottom->get_window()->create_cairo_context();
    cr->set_line_width(lineWidth);
    cr->set_source_rgb(defaultColor[0], defaultColor[1], defaultColor[2]);
    cr->scale(width, height);
    cr->move_to(0.0,inner);
    cr->line_to(1.0, inner);
    cr->move_to(0.0,middle);
    cr->line_to(1.0, middle);
    cr->move_to(0.0,outer);
    cr->line_to(1.0, outer);
    cr->stroke();

    cr = topViewBotRight->get_window()->create_cairo_context();
    cr->set_line_width(lineWidth);
    cr->set_source_rgb(defaultColor[0], defaultColor[1], defaultColor[2]);
    cr->scale(width, height);
    cr->move_to(inner, 0.0);
    cr->arc(0.0, 0.0, inner, 0, -M_PI/2);
    cr->move_to(middle,0.0);
    cr->arc(0.0, 0.0, middle, 0, -M_PI/2);
    cr->move_to(outer,0.0);
    cr->arc(0.0, 0.0, outer, 0, -M_PI/2);
    cr->stroke();

    cr = sideViewTop->get_window()->create_cairo_context();
    cr->set_line_width(lineWidth);
    cr->set_source_rgb(defaultColor[0], defaultColor[1], defaultColor[2]);
    cr->scale(width, height);
    cr->move_to(0.0,1.0 - outer);
    cr->line_to(1.0, 1.0 - outer);
    cr->move_to(0.0,1.0 - middle);
    cr->line_to(1.0, 1.0 - middle);
    cr->move_to(0.0,1.0 - inner);
    cr->line_to(1.0, 1.0 - inner);
    cr->stroke();

    cr = sideViewBottom->get_window()->create_cairo_context();
    cr->set_line_width(lineWidth);
    cr->set_source_rgb(defaultColor[0], defaultColor[1], defaultColor[2]);
    cr->scale(width, height);
    cr->move_to(0.0,inner);
    cr->line_to(1.0, inner);
    cr->move_to(0.0,middle);
    cr->line_to(1.0, middle);
    cr->move_to(0.0,outer);
    cr->line_to(1.0, outer);
    cr->stroke();
}
