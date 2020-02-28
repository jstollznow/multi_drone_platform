#include <string>
#include <iostream>
#include <vector>
#include <array>

#include "user_api.h"
#include "debugUI.h"

#define NODE_NAME "debugApplication"
#define NUM_WINDOWS 20
#define EXPANDED true

class debugApplication: public Gtk::Application {
    private:
        std::array<int,2> getWindowPosition(int droneNum, bool expanded);
        void showWindows();
    protected:
        ros::NodeHandle myAppNode;
        std::map<std::string, debugUI*> droneDebugUIs;
    public:
        debugApplication(std::vector<mdp_api::id> myDrones, int argc, char **argv, std::string appID = "debugApp");
};