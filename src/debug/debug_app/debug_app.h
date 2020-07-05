
#include "debug_window.h"

#include <string>
#include <iostream>
#include <vector>
#include <array>

#define NODE_NAME "debug_application"

class debug_app: public Gtk::Application {
    private:
        std::array<int, 2> get_window_position(int droneNum, bool expanded);
        void generate_session_folder();
    protected:
        std::map<std::string, debug_window*> droneDebugWindows;
    public:
        debug_app(std::vector<mdp::id> myDrones, int argc, char **argv, bool expanded);
};