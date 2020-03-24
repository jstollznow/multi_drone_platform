#include "teleop.h"

#define NODE_NAME "teleop"


int main(int argc, char **argv) {
    mdp::initialise(UPDATE_RATE, NODE_NAME);
    std::vector<mdp::id> controllableDrones = mdp::get_all_rigidbodies();

    auto it = controllableDrones.begin();
    while (it != controllableDrones.end()) {
        if (it->name.find("object") != std::string::npos) {
            auto toDelete = it;
            controllableDrones.erase(toDelete);
        }
        else {
            it = std::next(it, 1);
        }
    }

    teleop myTeleop(controllableDrones);

    for (auto drone: controllableDrones) {
        myTeleop.log(logger::INFO, std::to_string(drone.numericID) + ": " + drone.name.c_str());
    }

    if (controllableDrones.size() > 0) {
        myTeleop.run();
    }
    else {
        myTeleop.log(logger::ERROR, "No controllable objects");
    }

    mdp::terminate();

    return 0;
}