#include <string>
#include <iostream>
#include <vector>
#include <array>

#include "user_api.h"
#include "debug_window.h"

#define NODE_NAME "debug_application"
#define NUM_WINDOWS 20
#define EXPANDED true

class debug_app: public Gtk::Application {
    private:
        std::array<int, 2> get_window_position(int droneNum, bool expanded);
    protected:
        std::map<std::string, debug_window*> droneDebugWindows;
    public:
        debug_app(std::vector<mdp_api::id> myDrones, int argc, char **argv);
};