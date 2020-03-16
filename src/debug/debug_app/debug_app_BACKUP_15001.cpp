#include "debug_app.h"

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
        xSplit = 600;
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
<<<<<<< HEAD

void debug_app::show_windows() {
    for(auto& it : droneDebugWindows) {
        it.second->show();
    }
}

debug_app::debug_app(std::vector<mdp::id> myDrones, int argc, char **argv, std::string appID):Gtk::Application(argc, argv, appID) {
=======
debug_app::debug_app(std::vector<mdp::id> myDrones, int argc, char **argv)
:Gtk::Application(argc, argv, "debug_window.app") {
>>>>>>> origin/master
    bool expanded = EXPANDED;
    ros::init(argc, argv, NODE_NAME);

    for (size_t i = 0; i < myDrones.size(); i++) {
        debug_window *myWindow = 0;
        auto ui = Gtk::Builder::create_from_file(UI_PATH);
        ui->get_widget_derived("debugWindow", myWindow);
        myWindow->init(myDrones[i],this->get_window_position((int)i, expanded), expanded);
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