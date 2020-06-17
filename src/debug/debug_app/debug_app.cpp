#include "debug_app.h"
#include <ros/package.h>
#include <chrono>
#include <ctime>
std::array<int,2> debug_app::get_window_position(int droneNum, bool expanded) {
    int startPosX = 50;
    int startPosY = 30;
    int xSplit = 0;
    int ySplit = 0;
    int maxWindows = 0;
    int cols = 0;
    int rows = 0;
    int pos_x = 0;
    int pos_y = 0;
    int indent = 25;

    if (expanded) {
        xSplit = 900;
        ySplit = 550;
        maxWindows = 4;
        cols = 2;
        rows = cols;
    }
    else {
        xSplit = 630;
        ySplit = 350;
        maxWindows = 9;
        cols = 3;
        rows = cols;
    }
    
    int posDecider = droneNum % maxWindows;
    pos_x = startPosX + (droneNum/maxWindows) * indent;
    pos_y = startPosY + (droneNum/maxWindows) * indent;
    int x_index = (int)(posDecider) % cols;
    int y_index = (int)(posDecider) / rows;
    std::array<int, 2> position;
    position[0] = pos_x + xSplit * (x_index);
    position[1] = pos_y + ySplit * (y_index);
    return position;
}
void debug_app::generate_session_folder() {
    std::string path = ros::package::getPath("multi_drone_platform");
    std::time_t start = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string timeStr(16, '\0');
    std::strftime(&timeStr[0], timeStr.size(), "%d-%b_%T", std::localtime(&start));
    if (!path.empty()) {
        path = path + "/sessions/" +  timeStr.c_str() + "/";
        ros::param::set(SESSION_PARAM, path);
        auto command = "mkdir -p " + path;
        system(command.c_str());
    }
    else {
        ROS_WARN("Could not locate myself!?");
    }
}

debug_app::debug_app(std::vector<mdp::id> myDrones, int argc, char **argv, bool expanded)
:Gtk::Application(argc, argv, "debug_window.app") {
    ros::init(argc, argv, NODE_NAME);
    generate_session_folder();

    for (size_t i = 0; i < myDrones.size(); i++) {
        debug_window *myWindow = 0;
        std::string path = ros::package::getPath("multi_drone_platform") + UI_PATH;
        auto ui = Gtk::Builder::create_from_file(path);
        ui->get_widget_derived("debugWindow", myWindow);
        myWindow->init(myDrones[i], this->get_window_position((int)i, expanded), expanded);
        myWindow->windowSpinner.start();
        droneDebugWindows.insert(std::pair<std::string, debug_window*>(myDrones[i].name, myWindow));
    }

    this->signal_startup().connect([&]{
        for(auto& it : droneDebugWindows) {
            this->add_window(*(it.second));
        }
    });

    this->run();
}