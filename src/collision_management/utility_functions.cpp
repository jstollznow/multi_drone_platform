//
// Created by jacob on 20/5/20.
//
#include <cmath>

namespace utility_functions {
    template <class T>
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

    template <class T>
    bool coord_equality(T a, T b) {
        return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
    }

}
