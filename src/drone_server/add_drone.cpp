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

void do_add_by_prompts(ros::ServiceClient& client) {
    cout << "Adding new drones to connected drone server!" << endl;
    cout << "Input command: " << endl;
    while (true) {
        cout << "'NEW' for new drone, 'END' to exit: ";

        string initString;
        get_line(initString);
        if (initString == "END") {
            break;
        } else if (initString == "NEW") {
            string droneTag;
            cout << "Enter drone tag (name of the rigidbody on Optitrack) or vflie_## for a new vflie" << endl;
            cout << "Drone tag: ";
            get_line(droneTag);
            if (mdp_wrappers::get_drone_type_id(droneTag) == 0) {
                print_error("Unable to find drone type for the tag '" + droneTag + "'");
                print_error("Exiting drone addition");
                continue;
            }
            std::string dataDesc = mdp_wrappers::get_data_desc(droneTag);
            std::vector<std::string> results;
            boost::split(results, dataDesc, [](char c) { return c == ','; });
            for (std::string &arg : results) {
                cout << "Enter value for argument '" << arg << "': ";
                get_line(arg);
            }
            // send to add drone service with results as the arg list
            multi_drone_platform::add_drone msg;
            msg.request.arguments = results;
            msg.request.droneName = droneTag;
            if (!client.call(msg)) {
                print_error("Failed to contact drone server to make add drone request");
                return;
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
}

void print_help() {
    cout << "To add a drone through prompts, run this program with no arguments" << endl;
    cout << "To add a drone through arguments enter one of the following templates with the add drone program being 'add_drone'" << endl;
    cout << "add_drone <drone_tag> <..arguments>. Where drone_tag refers to the name of the drone rigidbody on Optitrack" << endl;
    for (const auto & it : droneTypeMap) {
        cout << "\tadd_drone " << it.first << "_##";
        std::string dataDesc = mdp_wrappers::get_data_desc(it.first);
        std::vector<std::string> results;
        if (!dataDesc.empty()) {
            boost::split(results, dataDesc, [](char c) { return c == ','; });
        }
        for (std::string& argName : results) {
            cout << " <" << argName << ">";
        }
        cout << endl;
    }
}

void do_add_by_arguments(int argc, char** argv, ros::ServiceClient& client) {
    std::string dataDesc = mdp_wrappers::get_data_desc(argv[1]);
    std::vector<std::string> results;
    if (!dataDesc.empty()) {
        boost::split(results, dataDesc, [](char c) { return c == ','; });
    }
    if (argc < (results.size()+2)) {
        print_error("Unable to add drone with tag '" + std::string(argv[1]) + "' as not enough aguments were passed. Expected " + std::to_string(results.size()) + " but only recieved " + std::to_string(argc - 2));
        return;
    }

    for (size_t i = 0; i < results.size(); i++) {
        // fill array with arguments
        results[i] = std::string(argv[i + 2]);
    }

    multi_drone_platform::add_drone msg;
    msg.request.droneName = argv[1];
    msg.request.arguments = results;

    if (!client.call(msg)) {
        print_error("Failed to contact drone server to make add drone request");
        return;
    } else {
        cout << endl;
        if (!msg.response.success) {
            print_error("Failed adding drone: " + msg.response.reason);
        } else {
            cout << BOLDGREEN << "Successfully added drone with tag: " << std::string(argv[1]) << RESET << endl;
        }
    }
}

int main(int argc, char** argv)
{
    if (argc > 1 && (strcmp(argv[1], "--help") == 0)) {
        print_help();
        return 0;
    }

    ros::init(argc, argv, "mdp_add_drone_node");
    ros::NodeHandle node;
    ros::ServiceClient client = node.serviceClient<multi_drone_platform::add_drone>("mdp/add_drone_srv", false);
    if (argc == 1) {
        do_add_by_prompts(client);
    } else {
        do_add_by_arguments(argc, argv, client);
    }

    cout << endl;
    return 0;
}