#include <string>
#include <vector>

#include "user_api.h"
#include "debug_app.h"


int main(int argc, char *argv[]) {

    mdp::initialise(10, "debug_app_test");

    /* Check for command line options, default expanded */
    bool expanded = true;
    if (argc > 1) {
        std::string state;
        state = argv[1];
        if (state == "expanded") expanded = true;
        if (state == "compressed") expanded = false;
    }

    /* create Gtk app and wait until it returns */
    auto app = debug_app(mdp::get_all_rigidbodies(), argc, argv, expanded);
}
