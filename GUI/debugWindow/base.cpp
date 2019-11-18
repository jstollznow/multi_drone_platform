#include <gtkmm/builder.h>
#include <gtkmm/application.h>
#include <gtkmm/applicationwindow.h>
#include <gtkmm/button.h>
#include <string>
#include <iostream>
#include <vector>
#include "../../include/user_api.h"
#include "debugUI.h"

// #define UI_PATH "/home/jacob/catkin_ws/src/multi_drone_platform/GUI/debugWindow/debug.ui"
// main window of the application

int main(int argc, char *argv[]) {
     // This creates an Gtk+ application with an unique application ID
    // Gtk::Main(argc, argv)::run();
    auto app = Gtk::Application::create(argc, argv);
    mdp_api::initialise(10);
    std::vector<debugUI*> myUIs;
    mdp_api::id myId;
    myId.name = "tubby_00";
    myId.numeric_id = 2;
    std::vector<mdp_api::id> myDrones;
    myDrones = mdp_api::get_all_rigidbodies();
    for (size_t i = 0; i < myDrones.size(); i++)
    {
        debugUI *myWindow = 0;
        auto ui = Gtk::Builder::create_from_file(UI_PATH);
        ui->get_widget_derived("debugWindow", myWindow);
        myWindow->init(myDrones[i]);
        myUIs.push_back(myWindow);
    }
    // app->signal_startup().connect()
    // app->add_window(*myWindow);
    // app->add_window(*myWindow2);
    // myWindow->set_application(app);
    // app->run();
    app->signal_startup().connect([&]{
        for(size_t i = 0; i < myUIs.size(); i++)
        {
            app->add_window(*myUIs[i]);
        }
        // app->add_window(*myWindow2);
        // app->add_window(*myWindow);
        });
    // myWindow2->show();
    return app->run();
}

void startup()
{

}
