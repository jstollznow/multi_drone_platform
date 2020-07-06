#include "rigidbody.h"

/* ensure the name of this file is identical to the first parameter of DRONE_WRAPPER, this will be the identifying tag of the new drone. If this
 * file is included in the /wrappers/ folder, then it will automatically be compiled with the drone server. When developing
 * a new drone wrapper, run catkin_make in the base folder of the catkin workspace to recompile the platform including the new drone.
 */

/* function documentation is available through compiling the platform's DoxyGen. The documentation for functions in a drone wrapper is then
 * available in the generated html under the 'rigidbody' class. (.../multi_drone_platform/docs/html/classrigidbody.html)
 */

/* The DRONE_WRAPPER(..) macro is ordered as follows, the first parameter is the identifying tag of the drone and all following
 * parameters represent an argument to be passed into the on_init(..) function at drone startup. i.e. DRONE_WRAPPER(object, argA, argB) will
 * mean that the drone's tag is 'object' and under the on_init(std::vector<std::string> args) function 'args[0]' will hold the result of
 * argA when the drone is declared, and 'args[1]' will represent argB. The values of these args are given when the drone is declared onto
 * the drone platform through the 'add_drone' platform program.
 */
class DRONE_WRAPPER(object)
    private:

    public:
    void on_init(std::vector<std::string> args) final {

    };

    void on_deinit() final {

    }

    void on_set_position(geometry_msgs::Vector3 pos, float yaw, float duration) final {

    }

    void on_set_velocity(geometry_msgs::Vector3 vel, float yawrate, float duration) final {

    }

    void on_motion_capture(const geometry_msgs::PoseStamped& msg) final {

    }
    
    void on_update() final {

    }

    void on_takeoff(float height, float duration) final {

    }

    void on_land(float duration) final {

    }

    void on_emergency() final {

    }
};


