#include "icp_object.h"

icp_object::icp_object(const std::string& rigidbody_name, ros::NodeHandle handle) {
    this->initSubscriber = handle.subscribe("/mocap/rigid_bodies/" + rigidbody_name + "/markers", 1, &icp_object::init_marker_callback, this);
    this->posePublisher = handle.advertise<geometry_msgs::PoseStamped>("/icp_impl/" + rigidbody_name + "/pose", 1);
}

icp_object::~icp_object() = default;

void icp_object::init_marker_callback(const natnet_msgs::MarkerList::ConstPtr &msg) {
    // get marker locations @TODO: may need to translate these to zero?
    this->markerTemplateLocations.resize(msg->positions.size());
    for (size_t i = 0; i < this->markerTemplateLocations.size(); i++) {
        this->markerTemplateLocations[i].x = msg->positions[i].x;
        this->markerTemplateLocations[i].y = msg->positions[i].y;
        this->markerTemplateLocations[i].z = msg->positions[i].z;
    }

    // close this ros subscriber, i.e. only do this once
    this->initSubscriber.shutdown();
}

const std::vector<geometry_msgs::Point> &icp_object::get_marker_template() const {
    return this->markerTemplateLocations;
}

bool icp_object::has_initialised() const {
    return !this->markerTemplateLocations.empty();
}
