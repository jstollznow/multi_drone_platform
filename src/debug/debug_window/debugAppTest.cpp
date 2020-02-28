#include <string>
#include <iostream>
#include <vector>
#include <array>

#include "user_api.h"
#include "debugApplication.h"


int main(int argc, char *argv[]) {

    // auto app = Gtk::Application::create(argc, argv);
    mdp_api::initialise(10);
    std::vector<mdp_api::id> myDrones;

    for (int i = 0; i < 10; i++) {
        mdp_api::id myId;
        myId.name = "vflie_" + std::to_string(i);
        myId.numeric_id = i;
        myDrones.push_back(myId);
    }
    
    auto app = debugApplication(myDrones, argc, argv, "debug.app");
}
