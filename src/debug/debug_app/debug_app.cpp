#include "debug_app.h"
#include <ros/package.h>
#include <chrono>
#include <ctime>

std::array<int,2> debug_app::get_window_position(int droneNum, bool expanded) {

    int startPosX = 50;
    int startPosY = 30;
    int xSplit = expanded ? 900 : 630;
    int ySplit = expanded ? 550 : 350;
    int maxWindows = expanded ? 4 : 9;
    int cols, rows;
    cols = rows = expanded ? 2 : 3;

    int indent = 25;
    
    int posDecider = droneNum % maxWindows;
    int pos_x = startPosX + (droneNum/maxWindows) * indent;
    int pos_y = startPosY + (droneNum/maxWindows) * indent;
    int x_index = posDecider % cols;
    int y_index = posDecider / rows;

    std::array<int, 2> position;
    position[0] = pos_x + xSplit * (x_index);
    position[1] = pos_y + ySplit * (y_index);
    return position;
}

void debug_app::generate_session_folder() {
    std::string path = ros::package::getPath("multi_drone_platform");

    /* Generate time for directory name */
    std::time_t start = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::string timeStr(16, '\0');
    std::strftime(&timeStr[0], timeStr.size(), "%d-%b_%T", std::localtime(&start));

    if (!path.empty()) {
        /* Create directory for session files */
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

    /* Create each window using debug.ui file template */
    for (size_t i = 0; i < myDrones.size(); i++) {
        debug_window *myWindow = 0;
        std::string path = ros::package::getPath("multi_drone_platform") + UI_PATH;
        auto ui = Gtk::Builder::create_from_file(path);
        ui->get_widget_derived("debugWindow", myWindow);
        myWindow->init(myDrones[i], this->get_window_position((int)i, expanded), expanded);

        /* Begin async event spinner */
        myWindow->windowSpinner.start();
        droneDebugWindows.insert(std::pair<std::string, debug_window*>(myDrones[i].name, myWindow));
    }

    /* Add each window to the application */
    this->signal_startup().connect([&]{
        for(auto& it : droneDebugWindows) {
            this->add_window(*(it.second));
        }
    });

    /* Run Gtk Application and listen for events */
    this->run();
}