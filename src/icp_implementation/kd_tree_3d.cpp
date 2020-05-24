#include "kd_tree_3d.h"

kd_tree_3d::kd_tree_3d() = default;

kd_tree_3d::kd_tree_3d(const std::vector<geometry_msgs::Point> &points) {
    // insert all points into the tree
    for (auto& point : points) {
        insert(point);
    }
}

kd_tree_3d::~kd_tree_3d() {
    deconstruct_rec(this->head);
}

void kd_tree_3d::insert(const geometry_msgs::Point &point) {
    // recursively add the node to the tree
    this->head = insert_rec({point.x, point.y, point.z}, this->head);
}

void kd_tree_3d::deconstruct_rec(kd_tree_node *node) {
    // recursively deletes all nodes in the tree
    if (node->left != nullptr) {
        deconstruct_rec(node->left);
    }
    if (node->right != nullptr) {
        deconstruct_rec(node->right);
    }
    delete node;
}

kd_tree_node* kd_tree_3d::insert_rec(const std::array<double, 3>& point, kd_tree_node *node, int current_dimension) {
    if (node == nullptr) {
        // if this node is a nullptr, create a new node here with point as data
        node = new kd_tree_node;
        node->data = point;
    } else if (node->data == point) {
        // node is a duplicate... this should not happen
        return node;
    } else {
        // node does not go here, traverse tree
        if (point[current_dimension] < node->data[current_dimension]) {
            node->left = insert_rec(point, node->left, (current_dimension+1) % 3);
        } else {
            node->right = insert_rec(point, node->right, (current_dimension+1) % 3);
        }
    }
    return node;
}

std::pair<Eigen::Vector3d, double> kd_tree_3d::find_nearest_neighbor(const Eigen::Vector3d &point) const {
    // perform recursive find nearest neighbor
    auto node_pair = nearest_neighbor_rec({point.x(), point.y(), point.z()}, this->head);

    // fill out geometry_msgs::Point pair with data
    std::pair<Eigen::Vector3d, double> ret;
    ret.first = {node_pair.first->data[0], node_pair.first->data[1], node_pair.first->data[2]};
    ret.second = node_pair.second;

    // return that pair
    return ret;
}

/* returns the squared euclidean distance from point a to point b */
inline double euc_dist_sq(const std::array<double, 3> &a, const std::array<double, 3> &b) {
    std::array<double, 3> d = { a[0] - b[0], a[1] - b[1], a[2] - b[2] };
    return std::pow(d[0], 2) + std::pow(d[1], 2) + std::pow(d[2], 2);
}

std::pair<kd_tree_node*, double> kd_tree_3d::nearest_neighbor_rec(const std::array<double, 3> &point, kd_tree_node *node, int current_dimension) {
    // return if this node is nullptr
    if (node == nullptr) {
        return {nullptr, 999999999.0};
    }

    // record the distance of this node to point
    std::pair<kd_tree_node*, double> this_node_dist = {node, euc_dist_sq(point, node->data)};

    // traverse the tree to find a possible lower distance point
    std::pair<kd_tree_node*, double> rec_closest;
    if (point[current_dimension] < node->data[current_dimension]) {
        rec_closest = nearest_neighbor_rec(point, node->left, (current_dimension+1) % 3);
    } else {
        rec_closest = nearest_neighbor_rec(point, node->right, (current_dimension+1) % 3);
    }

    // return the pair with the lower distance
    if (rec_closest.first != nullptr) {
        if (rec_closest.second < this_node_dist.second) {
            return rec_closest;
        }
    }
    return this_node_dist;
}
