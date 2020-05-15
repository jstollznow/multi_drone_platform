#ifndef SRC_ICP_OBJECT_H
#define SRC_ICP_OBJECT_H

#include <ros/ros.h>
#include <natnet_msgs/MarkerList.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

class icp_object {
public:
    ros::Publisher posePublisher;

    icp_object(const std::string& rigidbody_name, ros::NodeHandle handle);
    ~icp_object();

    // get this rigidbody's marker template
    const std::vector<geometry_msgs::Point>& get_marker_template() const;

    // check whether the icp object contains a valid marker template
    bool has_initialised() const;

private:
    std::vector<geometry_msgs::Point> markerTemplateLocations{};
    ros::Subscriber initSubscriber;

    // this is only used to get the initial marker positions for the rigidbody automatically
    void init_marker_callback(const natnet_msgs::MarkerList::ConstPtr& msg);
};


#endif //SRC_ICP_OBJECT_H
