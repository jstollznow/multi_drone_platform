#include "kd_tree_3d.h"
#include <iostream>

void add_point(std::vector<geometry_msgs::Point>& points, double x, double y, double z) {
    geometry_msgs::Point p;
    p.x = x; p.y = y; p.z = z;
    points.push_back(p);
}

int main() {
    std::vector<geometry_msgs::Point> points;
    add_point(points, 0.0, 0.0, 0.0);
    add_point(points, 1.0, 0.0, 0.0);
    add_point(points, 2.0, 0.0, 0.0);

    kd_tree_3d t(points);

    geometry_msgs::Point p;
    p.x = 0.16; p.y = 0.5; p.z = 1.0;

    auto ret = t.find_nearest_neighbor(p);
    std::cout << ret.first.x << "," << ret.first.y << "," << ret.first.z << " d:" << ret.second << std::endl;

    return 0;
}