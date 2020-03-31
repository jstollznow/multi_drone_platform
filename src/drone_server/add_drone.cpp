#include <ros/ros.h>
#include <boost/algorithm/string/split.hpp>
#include "wrappers.h"
#include "multi_drone_platform/add_drone.h"

#define RESET   "\033[0m"
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */


using namespace std;

void print_error(const std::string& msg) {
    cout << (BOLDRED + msg + RESET) << endl;
}

void get_line(std::string& str) {
    str = "";
    while (str.empty()) {
        std::getline(std::cin, str);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mdp_add_drone_node");
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<multi_drone_platform::add_drone>("mdp/add_drone_srv", false);
    cout << "Adding new drones to connected drone server!" << endl;
    cout << "input command: " << endl;
    while (true) {
        cout << "'NEW' for new drone, 'END' to exit: ";

        string initString;
        get_line(initString);
        if (initString == "END") {
            break;
        } else if (initString == "NEW") {
            string droneTag;
            cout << "enter drone tag (name of the rigidbody on Optitrack) or vflie_## for a new vflie" << endl;
            cout << "drone tag: ";
            get_line(droneTag);
            if (mdp_wrappers::get_drone_type_id(droneTag) == 0) {
                print_error("Unable to find drone type for the tag '" + droneTag + "'");
                print_error("Exiting drone addition");
                continue;
            }
            std::string dataDesc = mdp_wrappers::get_data_desc(droneTag);
            std::vector<std::string> results;
            boost::split(results, dataDesc, [](char c){return c == ',';});
            for (std::string& arg : results) {
                cout << "enter value for argument '" << arg << "': ";
                get_line(arg);
            }
            // send to add drone service with results as the arg list
            multi_drone_platform::add_drone msg;
            msg.request.arguments = results;
            msg.request.droneName = droneTag;
            if (!client.call(msg)) {
                print_error("Failed to contact drone server to make add drone request");
                return 0;
            } else {
                cout << endl;
                if (!msg.response.success) {
                    print_error("Failed adding drone: " + msg.response.reason);
                } else {
                    cout << BOLDGREEN << "Successfully added drone with tag: " << droneTag << RESET << endl;
                }
            }
        } else {
            print_error("Please enter a valid command");
        }
    }

    cout << endl;
    return 0;
}