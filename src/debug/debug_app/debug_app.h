
#include "debug_window.h"

#include <string>
#include <iostream>
#include <vector>
#include <array>

#define NODE_NAME "debug_application"

class debug_app: public Gtk::Application {
private:
    /**
     * Used to determine the position on the screen where the window should be created, this is dependent on whether
     * the window is expanded or compressed and assumes a full HD display. Windows are offset by a small amount when
     * overlapping results due to the number of drones. In the compressed format, 9 windows in a 3 by 3 format can be
     * seen at one time. In the expanded format, 6 windows in a 2 by 2 format are displayed with no overlap.
     * @param droneNum Used to indicate position in window grid positioning system.
     * @param expanded To determine the launch option of each window.
     * @return Returns an array containing the top corner x and y coordinates of the specified live view window.
     */
    std::array<int, 2> get_window_position(int droneNum, bool expanded);

    /**
     * This method generates a session folder according to the time which the session was started. This is useful for
     * post-flight analysis and the time reference enables the user to quickly navigate to the appropriate folder. This
     * folder path is then saved as a ROS parameter and used across other aspects of the platform.
     */
    void generate_session_folder();
protected:
    /**
     * Map containing the window associated with each drone passed into the constructor, most commonly this includes
     * all rigidbodies in the platform, but can be a selection of them.
     */
    std::map<std::string, debug_window*> droneDebugWindows;
public:
    /**
     * The constructor creates a Live View window for each of the drones in the myDrones vector. Each window will be
     * either expanded or compressed according to the expanded boolean parameter. Each of the windows are then displayed
     * according to the window position obtained using the appropriate function above.
     * @param myDrones Vector of drones for which you would like live view windows.
     * @param argc Command line arguments to pass onto ROS.
     * @param argv Command line arguments to pass onto ROS.
     * @param expanded Boolean variable indicating whether the Live View windows should be expanded or compressed.
     */
    debug_app(std::vector<mdp::id> myDrones, int argc, char **argv, bool expanded);

};