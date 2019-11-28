#include <string>
#include <iostream>
#include <vector>
#include <array>

#include "../../include/user_api.h"
#include "debugUI.h"

#define NUM_WINDOWS 20
#define EXPANDED true

// itds

// #define UI_PATH "/home/jacob/catkin_ws/src/multi_drone_platform/GUI/debugWindow/debug.ui"
// main window of the application
std::array<int,2> getPosition(int droneNum, bool expanded)
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
int main(int argc, char *argv[]) {

    auto app = Gtk::Application::create(argc, argv);

    // mdp_api::initialise(10);
    // ros::init(argc, argv, "debugNode");
    std::vector<mdp_api::id> myDrones;
    // myDrones = mdp_api::get_all_rigidbodies();

    std::vector<debugUI*> myUIs;
    
    mdp_api::id myId0;
    myId0.numeric_id = 0;
    for(int i = 0; i < NUM_WINDOWS; i++)
    {
        myId0.name = "vflie_" + std::to_string(i);
        myDrones.push_back(myId0);
    }


    bool expanded = EXPANDED;
    for (size_t i = 0; i < myDrones.size(); i++)
    {
        debugUI *myWindow = 0;
        auto ui = Gtk::Builder::create_from_file(UI_PATH);
        ui->get_widget_derived("debugWindow", myWindow);
        myWindow->init(myDrones[i],getPosition((int)i, expanded), expanded);
        myUIs.push_back(myWindow);
    }

    app->signal_startup().connect([&]{
        
        for(size_t i = 0; i < myUIs.size(); i++)
        {
            app->add_window(*myUIs[i]);
            // myUIs[i]->mySpin.start();
        }   

        });
    app->run();
    
    // ros::Rate loop_rate(UPDATE_RATE);
    // while (ros::ok())
    // {
    //     ros::spinOnce();
        
    //     loop_rate.sleep();
    // }
}
