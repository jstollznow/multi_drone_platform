#include <string>
#include <iostream>
#include <vector>
#include <array>

#include "user_api.h"
#include "debug_app.h"


int main(int argc, char *argv[]) {

    mdp::initialise(10, "debug_app_test");

    bool expanded = true;
    if (argc > 1) {
        std::string state;
        state = argv[1];
        if (state == "expanded") expanded = true;
        if (state == "compressed") expanded = false;
    }



    // for potential testing
    std::vector<mdp::id> myDrones;

    for (int i = 0; i < 10; i++) {
        mdp::id myId;
        myId.name = "vflie_" + std::to_string(i);
        myId.numericID = i;
        myDrones.push_back(myId);
    }

    auto app = debug_app(mdp::get_all_rigidbodies(), argc, argv, expanded);
}
