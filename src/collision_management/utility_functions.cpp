//
// Created by jacob on 20/5/20.
//
#include <cmath>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Point.h"

namespace utility_functions {
    template<class T>
    double distance_between(T a, T b) {
        double dist = 0.0f;
        dist += (a.x - b.x) * (a.x - b.x);
        dist += (a.y - b.y) * (a.y - b.y);
        dist += (a.z - b.z) * (a.z - b.z);
        return std::sqrt(dist);
    }

    template<class T>
    T difference(T a, T b) {
        T ret;
        ret.x = a.x - b.x;
        ret.y = a.y - b.y;
        ret.z = a.z - b.z;
        return ret;
    }

    template<class T>
    T add_vec3_or_point(T a, T b) {
        T ret;
        ret.x = a.x + b.x;
        ret.y = a.y + b.y;
        ret.z = a.z + b.z;
        return ret;
    }

    template<class T>
    T multiply_by_constant(T a, double multiple) {
        T ret;
        ret.x = a.x * multiple;
        ret.y = a.y * multiple;
        ret.z = a.z * multiple;
        return ret;
    }
    template<class T>
    double magnitude(T a) {
        double sum = a.x * a.x;
        sum += a.y * a.y;
        sum += a.z * a.z;
        return std::sqrt(sum);
    }
    template<class T>
    bool coord_equality(T a, T b) {
        return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
    }

    geometry_msgs::Vector3 point_to_vec3(geometry_msgs::Point point) {
        geometry_msgs::Vector3 vec3;
        vec3.x = point.x;
        vec3.y = point.y;
        vec3.z = point.z;
        return vec3;
    }


}
