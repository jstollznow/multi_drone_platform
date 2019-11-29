#include <string>
#include <iostream>
#include <vector>
#include <array>

#include "../../include/user_api.h"
#include "debugApplication.h"


int main(int argc, char *argv[]) {

    // auto app = Gtk::Application::create(argc, argv);
    mdp_api::initialise(10);

    auto app = debugApplication(mdp_api::get_all_rigidbodies(), argc, argv, "DebugApp");
}
