#include "icp_impl.h"
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

icp_impl::icp_impl(const std::vector<rigidbody *> *rigidbodyListPtr, ros::NodeHandle& nodeHandle) {
    this->rigidbodyList = rigidbodyListPtr;
    this->markerCloudSubscriber = nodeHandle.subscribe("/markers/vis", 1, &icp_impl::marker_cloud_callback, this);
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

        // ignore vflies (special case, vflies produce their own poses)
        // comment this out to test ICP using a vflie, also enable publishing of markers from vflie
        if (rigidbody->isVflie) {
            continue;
        }

        // if the icp object has not been initialised, ignore it
        if (!rigidbody->icpObject.has_initialised()) {
            continue;
        }

        // do icp implementation to get the new pose
        Eigen::Quaterniond q;
        q.setIdentity();
        geometry_msgs::Pose p;
        p.orientation.w = q.w();p.orientation.x = q.x();p.orientation.y = q.y();p.orientation.z = q.z();
        auto newPose = icp_impl::perform_icp(rigidbody->icpObject.get_marker_template(), p, pointCloudTree);

        // create a PoseStamped from generated pose and timeNow
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header = premadeHeader;
        poseStamped.pose = newPose;

        // publish stamped pose for this rigidbody
        rigidbody->icpObject.posePublisher.publish(poseStamped);
    }

    std::cout << "Average iter time: " << (this->TiterationTime / this->count) << std::endl;
}

#define ICP_MAX_ITERATIONS 10
#define ICP_ERROR_THRESHOLD 0.0001
geometry_msgs::Pose icp_impl::perform_icp(const std::vector<geometry_msgs::Point>& markerTemplate, geometry_msgs::Pose poseEstimate, const kd_tree_3d& pointCloudTree)
{
    // @TODO... test
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

    int iteration = 0;
    double error_value = 9999.0;

    /* apply estamate pose on marker template */
    Eigen::Quaterniond estimateQuat(poseEstimate.orientation.w, poseEstimate.orientation.x, poseEstimate.orientation.y, poseEstimate.orientation.z);
    Eigen::Vector3d estimateTranslation(poseEstimate.position.x, poseEstimate.position.y, poseEstimate.position.z);

    ros::Time t1;

    while (iteration < ICP_MAX_ITERATIONS && error_value > ICP_ERROR_THRESHOLD) {
        t1 = ros::Time::now();
        /* convert marker template to eigen vectors */
        std::vector<Eigen::Vector3d> poseMarkers;
        for (const geometry_msgs::Point& p : markerTemplate) {
            Eigen::Vector3d point(p.x, p.y, p.z);
            poseMarkers.emplace_back(point);
        }

        /* transform marker template by estimate rotation */
        for (Eigen::Vector3d& point : poseMarkers) {
            /* rotate about estimate rotation */
            point = estimateQuat * point;
        }


        double averageDistanceSq = 0.0;
        std::vector<Eigen::Vector3d> dstPoints; dstPoints.reserve(poseMarkers.size());
        Eigen::Vector3d dstCentroid(0.0, 0.0, 0.0);
        Eigen::Vector3d srcCentroid = estimateTranslation;
        for (const Eigen::Vector3d& poseMarker : poseMarkers) {
            auto closestPoint = pointCloudTree.find_nearest_neighbor(poseMarker);
            dstPoints.emplace_back(closestPoint.first);
            dstCentroid += closestPoint.first;

            /* add to running average */
            averageDistanceSq += closestPoint.second;
        }

        /* finalise average distance squared */
        dstCentroid /= poseMarkers.size();
        averageDistanceSq /= poseMarkers.size();

        /* bring destination points to origin */
        for (Eigen::Vector3d& dstPoint : dstPoints) {
            dstPoint -= dstCentroid;
        }

        double Sxx = 0.0, Sxy = 0.0, Sxz = 0.0, Syx = 0.0, Syy = 0.0, Syz = 0.0, Szx = 0.0, Szy = 0.0, Szz = 0.0;
        for (size_t i = 0; i < dstPoints.size(); i++) {
            Sxx += (poseMarkers[i].x() * dstPoints[i].x());
            Sxy += (poseMarkers[i].x() * dstPoints[i].y());
            Sxz += (poseMarkers[i].x() * dstPoints[i].z());

            Syx += (poseMarkers[i].y() * dstPoints[i].x());
            Syy += (poseMarkers[i].y() * dstPoints[i].y());
            Syz += (poseMarkers[i].y() * dstPoints[i].z());

            Szx += (poseMarkers[i].z() * dstPoints[i].x());
            Szy += (poseMarkers[i].z() * dstPoints[i].y());
            Szz += (poseMarkers[i].z() * dstPoints[i].z());
        }

        Eigen::Matrix4d Nmat;
        Nmat << (Sxx + Syy + Szz),          (Syz - Szy),            (Szx - Sxz),                (Sxy - Syx),
                (Syz - Szy),                (Sxx - Syy - Szz),      (Sxy + Syx),                (Szx + Sxz),
                (Szx - Sxz),                (Sxy + Syx),            (-Sxx + Syy - Szz),         (Syz + Szy),
                (Sxy - Syx),                (Szx + Sxz),            (Syz + Szy),                (-Sxx - Syy + Szz);

        Eigen::EigenSolver<Eigen::Matrix4d> es;
        es.compute(Nmat, true);

        int max_index = 0;
        double max_num = 0.0;
        for (int i = 0; i < es.eigenvalues().size(); i++) {
            if (es.eigenvalues()[i].real() > max_num) {
                max_index = i;
                max_num = es.eigenvalues()[i].real();
            }
        }

        auto maxEigenVector = es.eigenvectors().col(max_index);
        Eigen::Quaterniond q;
        q.w() = maxEigenVector[0].real();
        q.x() = maxEigenVector[1].real();
        q.y() = maxEigenVector[2].real();
        q.z() = maxEigenVector[3].real();
        estimateQuat = q * estimateQuat;

        estimateTranslation += (dstCentroid - srcCentroid);

        /* apply error metric and increment iteration */
        error_value = averageDistanceSq;
        iteration++;

        this->TiterationTime += ros::Time().now().toSec() - t1.toSec();
        this->count += 1.0;
    }

    if (iteration >= ICP_MAX_ITERATIONS) {
        std::cout << "hit max iterations..., error: " << error_value << std::endl;
    } else {
        std::cout << "finished in iterations: " << iteration << std::endl;
    }

    /* fill poseEstimate with new data and return */
    poseEstimate.orientation.w = estimateQuat.w();
    poseEstimate.orientation.x = estimateQuat.x();
    poseEstimate.orientation.y = estimateQuat.y();
    poseEstimate.orientation.z = estimateQuat.z();
    poseEstimate.position.x = estimateTranslation.x();
    poseEstimate.position.y = estimateTranslation.y();
    poseEstimate.position.z = estimateTranslation.z();

    return poseEstimate;
}
