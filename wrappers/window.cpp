#include "rigidbody.h"
#include <Eigen/Geometry>

struct window_bounds {
    std::array<double, 3> min;
    std::array<double, 3> max;
};

inline geometry_msgs::Point create_point(double x, double y, double z) {
    geometry_msgs::Point res;
    res.x = x;
    res.y = y;
    res.z = z;
    return res;
}

class DRONE_WRAPPER(window)
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

    /**
     * checks whether the point is within the bounds of the window. This is generally meant to be used where input
     * point is found via a call to get_closest_point_on_window_plane() such that input point is on the plane of the window.
     * @param point a point anywhere in space
     * @return whether this point is within the bounding box of the window or not via an AABB test.
     */
    bool is_point_within_window(const geometry_msgs::Point& point) const {
        auto bounds = this->get_bounds();
        bool within_window_bounds = true;
        if (point.x < bounds.min[0] || point.x > bounds.max[0]) within_window_bounds = false;
        if (point.y < bounds.min[1] || point.y > bounds.max[1]) within_window_bounds = false;
        if (point.z < bounds.min[2] || point.z > bounds.max[2]) within_window_bounds = false;
        return within_window_bounds;
    }

    /**
     * returns the bounds of the window in world coordinates (min, max) for (x, y, z)
     */
    window_bounds get_bounds() const {
        window_bounds bounds_template;
        bounds_template.min = {0.0, -0.32, -0.17};
        bounds_template.max = {0.0, 0.32, 0.17};

        auto pose = get_current_pose();
        Eigen::Vector3d current_position = {pose.position.x, pose.position.y, pose.position.z};
        Eigen::Quaterniond q = {pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z};
        Eigen::Vector3d min = {bounds_template.min[0], bounds_template.min[1], bounds_template.min[2]};
        Eigen::Vector3d max = {bounds_template.max[0], bounds_template.max[1], bounds_template.max[2]};
        min = (q * min) + current_position;
        max = (q * max) + current_position;

        bounds_template.min = {min.x(), min.y(), min.z()};
        bounds_template.max = {max.x(), max.y(), max.z()};
        return bounds_template;
    };

    /**
     * returns the closest point on the window's plane to the given point p
     * @param p the given point
     * @return the closest point on the window's plane
     */
    geometry_msgs::Point get_closest_point_on_window_plane(const geometry_msgs::Point& p) {
        auto bounds = get_bounds();

        auto pointA = Eigen::Vector3d(bounds.min[0], bounds.min[1], bounds.min[2]);
        auto pointB = Eigen::Vector3d(bounds.max[0], bounds.max[1], bounds.max[2]);
        auto pointC = Eigen::Vector3d(bounds.min[0], bounds.min[1], bounds.max[2]);
        Eigen::Hyperplane<double, 3> plane = Eigen::Hyperplane<double, 3>::Through(pointA, pointB, pointC);

        auto closestPoint = plane.projection(Eigen::Vector3d(p.x, p.y, p.z));
        return create_point(closestPoint.x(), closestPoint.y(), closestPoint.z());
    }
};

