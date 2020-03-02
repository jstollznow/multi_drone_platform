#include <string>
#include <iostream>
#include <vector>
#include <array>

#include "user_api.h"
#include "debug_app.h"


int main(int argc, char *argv[]) {

    mdp::initialise(10);
    std::vector<mdp::id> myDrones;

    for (int i = 0; i < 10; i++) {
        mdp::id myId;
        myId.name = "vflie_" + std::to_string(i);
        myId.numericID = i;
        myDrones.push_back(myId);
    }
    
    // name must have '.' in it, else will throw runtime error
    auto app = debug_app(myDrones, argc, argv, "debug.app");
}
