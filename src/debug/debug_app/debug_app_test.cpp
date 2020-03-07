#include <string>
#include <iostream>
#include <vector>
#include <array>

#include "user_api.h"
#include "debug_app.h"


int main(int argc, char *argv[]) {

    mdp_api::initialise(10);
    std::vector<mdp_api::id> myDrones;

    for (int i = 0; i < 10; i++) {
        mdp_api::id myId;
        myId.name = "vflie_" + std::to_string(i);
        myId.numericID = i;
        myDrones.push_back(myId);
    }
    
    // name must have '.' in it, else will throw runtime error
    auto app = debug_app(mdp_api::get_all_rigidbodies(), argc, argv);
}
