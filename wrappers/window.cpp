#include "rigidbody.h"
#include <Eigen/Dense>

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

/**
 * math and util to represent a 3d plane
 */
class plane_3d {
    geometry_msgs::Point position;
    geometry_msgs::Point normal;

    inline geometry_msgs::Point cross_prod(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
        geometry_msgs::Point res;
        res.x = (a.y * b.z) - (a.z * b.y);
        res.y = (a.z * b.x) - (a.x * b.z);
        res.z = (a.x * b.y) - (a.y - b.x);
        return res;
    }

    inline geometry_msgs::Point minus(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
        geometry_msgs::Point res;
        res.x = a.x - b.x;
        res.y = a.y - b.y;
        res.z = a.z - b.z;
        return res;
    }

    inline double magnitude(const geometry_msgs::Point& p) {
        return std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2) + std::pow(p.z, 2));
    }

    inline geometry_msgs::Point div(const geometry_msgs::Point& p, double d) {
        geometry_msgs::Point res;
        res.x = p.x / d;
        res.y = p.y / d;
        res.z = p.z / d;
        return res;
    }

public:
    plane_3d(geometry_msgs::Point a, geometry_msgs::Point b, geometry_msgs::Point c) {
        this->position = a;
        auto dir = cross_prod(minus(b, a), minus(c, a));
        this->normal = div(dir, magnitude(dir));
    }
    ~plane_3d() {}

    geometry_msgs::Point get_closest_point_on_plane(const geometry_msgs::Point& from_point) const {
        // @TODO: should probably check this math
        double c = (normal.x * position.x) + (normal.y * position.y) + (normal.z * position.z);
        double k = (-c - (normal.x * from_point.x) - (normal.y * from_point.y) - (normal.z * from_point.z)) / ((normal.x * normal.x) + (normal.y * normal.y) + (normal.z * normal.z));

        geometry_msgs::Point res;
        res.x = normal.x * k + from_point.x;
        res.y = normal.y * k + from_point.y;
        res.z = normal.z * k + from_point.z;
        return res;
    }
};

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
        plane_3d windowPlane(
                create_point(bounds.min[0], bounds.min[1], bounds.min[2]),
                create_point(bounds.max[0], bounds.max[1], bounds.max[2]),
                create_point(bounds.min[0], bounds.min[1], bounds.max[2]));
        return windowPlane.get_closest_point_on_plane(p);
    }
};

