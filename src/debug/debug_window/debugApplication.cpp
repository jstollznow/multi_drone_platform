#include "debugApplication.h"

std::array<int,2> debugApplication::getWindowPosition(int droneNum, bool expanded)
{
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

    if (expanded)
    {
        xSplit = 900;
        ySplit = 500;
        maxWindows = 4;
        cols = 2;
        rows = cols;
    }
    else
    {
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

debugApplication::debugApplication(std::vector<mdp_api::id> myDrones, int argc, char **argv, std::string appID):Gtk::Application(argc, argv, appID)
{
    bool expanded = EXPANDED;
    // node stuff, create myDrones
    ros::init(argc, argv, NODE_NAME);

    for (size_t i = 0; i < droneDebugUIs.size(); i++)
    {
        debugUI *myWindow = 0;
        auto ui = Gtk::Builder::create_from_file(UI_PATH);
        ui->get_widget_derived("debugWindow", myWindow);
        myWindow->init(myDrones[i],this->getWindowPosition((int)i, expanded), expanded);
        droneDebugUIs.insert(std::pair<std::string, debugUI*>(myDrones[i].name, myWindow));
    }
    // Glib::signal_timeout().connect(sigc::ptr_fun(&timeout_handler_1), 100);
    // sigc::connection Glib::SignalTimeout::connect(const sigc::slot<bool>& slot,
    //                                   unsigned int interval, int priority = Glib::PRIORITY_DEFAULT);

    this->signal_startup().connect([&]{
        auto it = droneDebugUIs.begin();
        while (it != droneDebugUIs.end())
        {
            this->add_window(*(it->second));  
            it++;
        }
    });
}