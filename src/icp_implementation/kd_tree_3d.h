#ifndef SRC_KD_TREE_3D_H
#define SRC_KD_TREE_3D_H
#include "geometry_msgs/Point.h"
#include <array>

struct kd_tree_node {
    std::array<double, 3> data{};
    kd_tree_node* left = nullptr;
    kd_tree_node* right = nullptr;
};

class kd_tree_3d {
private:
    kd_tree_node* head = nullptr;

    static kd_tree_node* insert_rec(const std::array<double, 3>& point, kd_tree_node* node, int current_dimension = 0);
    static void deconstruct_rec(kd_tree_node* node);
    static std::pair<kd_tree_node*, double> nearest_neighbor_rec(const std::array<double, 3>& point, kd_tree_node* node, int current_dimension = 0);

public:
    kd_tree_3d();
    explicit kd_tree_3d(const std::vector<geometry_msgs::Point>& points);
    ~kd_tree_3d();

    /* inserts a point into the tree */
    void insert(const geometry_msgs::Point& point);

    /* finds and returns the nearest node in the tree to point and the euclidean distance between the two */
    std::pair<geometry_msgs::Point, double> find_nearest_neighbor(const geometry_msgs::Point &point);
};

/* built from https://www.cs.cmu.edu/~ckingsf/bioinfo-lectures/kdtrees.pdf */

#endif //SRC_KD_TREE_3D_H
