#include <string>
#include <iostream>
#include <vector>
#include <array>

#include "user_api.h"
#include "debug_window.h"

#define NODE_NAME "debugApplication"
#define NUM_WINDOWS 20
#define EXPANDED false

class commander: public Gtk::Application {
    private:
    protected:
        ros::NodeHandle commanderNode;
    public:
        commander(int argc = NULL, char **argv = NULL, std::string appID = "commander.app");
};