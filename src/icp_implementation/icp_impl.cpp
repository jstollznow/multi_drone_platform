#include "icp_impl.h"

icp_impl::icp_impl(const std::vector<rigidbody *> *rigidbodyListPtr, ros::NodeHandle& nodeHandle) {
    this->rigidbodyList = rigidbodyListPtr;
    this->markerCloudSubscriber = nodeHandle.subscribe("~/markers/vis", 1, &icp_impl::marker_cloud_callback, this);
}

icp_impl::~icp_impl() = default;

void icp_impl::marker_cloud_callback(const visualization_msgs::Marker::ConstPtr& msg)
{
    // premake the stamped header
    std_msgs::Header premadeHeader;
    premadeHeader.stamp = ros::Time::now();
    premadeHeader.frame_id = "world";

    kd_tree_3d pointCloudTree(msg->points);

    for (auto rigidbody : *this->rigidbodyList) {
        // if this rigidbody does not exist (deleted or uninitialised), ignore it
        if (rigidbody == nullptr) {
            continue;
        }

        // if the icp object has not been initialised, ignore it
        if (!rigidbody->icpObject.has_initialised()) {
            continue;
        }

        // do icp implementation to get the new pose
        auto newPose = this->perform_icp(rigidbody->icpObject.get_marker_template(), rigidbody->get_current_pose(), pointCloudTree);

        // create a PoseStamped from generated pose and timeNow
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header = premadeHeader;
        poseStamped.pose = newPose;

        // publish stamped pose for this rigidbody
        rigidbody->icpObject.posePublisher.publish(poseStamped);
    }
}

geometry_msgs::Pose icp_impl::perform_icp(std::vector<geometry_msgs::Point> markerTemplate, const geometry_msgs::Pose& initialEstimate, const kd_tree_3d& pointCloudTree)
{
    // @TODO...

    /*
     * 1. Initialise markerTemplate to initial estimate (i.e. the rigidbody's last pose).
     *      This will probably be a transformation of all marker points by the pose values
     *
     * 2. compute the closest points from markerCloud to the marker template locations.
     *      This will probably be done by placing markerCloud into a kd tree (3d).
     *      Kd tree should probably be passed in to this function and computed once per frame.
     *
     * 3. Compute the transformation to minimize distance to each closest point.
     *      This will most likely be done using least squares optimisation.
     *
     * 4. Apply this transformation to markerTemplate and determine how far off we are (i.e. the error value).
     *
     * 5. if the error value is not less than tolerance level, repeat steps 2 to 5.
     *
     * 6. return the resulting transformation as a pose from origin.
    */

    return initialEstimate;
}
