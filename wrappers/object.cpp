#include "rigidbody.h"

/* ensure the name of this file is identical to the name of the class, this will be the tag of the new drone. If this
 * file is included in the /wrappers/ folder, then it will automatically be compiled with the drone server. When developing
 * a new drone wrapper, run catkin_make in the base folder of the catkin workspace to recompile the platform including the new drone.
 */
class DRONE_WRAPPER(object)
    private:

    public:
    void on_init(std::vector<std::string> args) final {

    };

    void on_deinit() final {

    }

    void on_set_position(geometry_msgs::Vector3 pos, float yaw, float duration, bool isRelative) final {

    }

    void on_set_velocity(geometry_msgs::Vector3 vel, float yawrate, float duration, bool relativeHeight) final {

    }

    void on_motion_capture(geometry_msgs::PoseStamped msg) final {

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


