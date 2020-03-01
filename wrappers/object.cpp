#include "rigidbody.h"

class object : public rigidbody {
    private:

    public:
    object(std::string tag, uint32_t id) : rigidbody(tag, id) {

    };

    ~object() {

    }

    void on_set_position(geometry_msgs::Vector3 pos, float yaw, float duration, bool isRelative) override {

    }

    void on_set_velocity(geometry_msgs::Vector3 vel, float yawrate, float duration, bool isRelative) override {

    }

    void on_motion_capture(const geometry_msgs::PoseStamped::ConstPtr& msg) {

    }
    
    void on_update() override {

    }

    void on_takeoff(float height, float duration) override {

    }

    void on_land(float duration) override {

    }

    void on_emergency() override {

    }
};


