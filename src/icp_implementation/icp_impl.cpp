#include "icp_impl.h"
#include <Eigen/Dense>

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
        auto newPose = icp_impl::perform_icp(rigidbody->icpObject.get_marker_template(), rigidbody->get_current_pose(), pointCloudTree);

        // create a PoseStamped from generated pose and timeNow
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header = premadeHeader;
        poseStamped.pose = newPose;

        // publish stamped pose for this rigidbody
        rigidbody->icpObject.posePublisher.publish(poseStamped);
    }
}

inline Eigen::Matrix3d construct_rotation_matrix(double a, double b, double y) {
    double cosA = cos(a), cosB = cos(b), cosY = cos(y);
    double sinA = sin(a), sinB = sin(b), sinY = sin(y);
    Eigen::Matrix3d r;
    r <<    cosY*cosB, -sinY*cosA + cosY*sinB*sinA, sinY*sinA + cosY*sinB*cosA,
            sinY*cosB, -cosY*cosA + sinY*sinB*sinA, -cosY*sinA + sinY*sinB*cosA,
            -sinB, cosB*sinA, cosB*cosA;
    return r;
}

#define ICP_MAX_ITERATIONS 10
#define ICP_ERROR_THRESHOLD 0.1
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

    while (iteration < ICP_MAX_ITERATIONS && error_value > ICP_ERROR_THRESHOLD) {
        /* convert marker template to eigen vectors */
        std::vector<Eigen::Vector3d> poseMarkers;
        std::vector<Eigen::Vector3d> normals;
        for (const geometry_msgs::Point& p : markerTemplate) {
            Eigen::Vector3d point(p.x, p.y, p.z);
            poseMarkers.emplace_back(point);
            normals.emplace_back(point.normalized());
        }

        /* transform marker template by estimate */
        for (Eigen::Vector3d& point : poseMarkers) {
            /* rotate about estimate rotation */
            point = estimateQuat * point;

            /* then translate */
            point += estimateTranslation;
        }

        /* compute closest points, centroids, and average distance between point pairs (squared) */
        Eigen::MatrixXd matA;
        matA.resize(poseMarkers.size(), 6);
        Eigen::VectorXd vecB;
        vecB.resize(poseMarkers.size());
        double averageDistanceSq = 0.0;
        for (int i = 0; i < poseMarkers.size(); i++) {
            auto closestPoint = pointCloudTree.find_nearest_neighbor(poseMarkers[i]);

            /* add row to matA */
            std::array<double, 3> a{};
            a[0] = normals[i].z() * closestPoint.first.y() - normals[i].y() * closestPoint.first.z();
            a[1] = normals[i].x() * closestPoint.first.z() - normals[i].z() * closestPoint.first.x();
            a[2] = normals[i].y() * closestPoint.first.x() - normals[i].x() * closestPoint.first.y();
            matA.row(i) << a[0], a[1], a[2], normals[i].x(), normals[i].y(), normals[i].z();

            /* add row to vecB */
            vecB[i] = normals[i].dot(poseMarkers[i]) - normals[i].dot(closestPoint.first);

            /* add to running average */
            averageDistanceSq += closestPoint.second;
        }
        /* finalise average distance squared */
        averageDistanceSq /= poseMarkers.size();

        /* perform SVD using matA and vecB (MatA * x - vecB) */
        Eigen::VectorXd svdResults = matA.bdcSvd().solve(vecB);

        /* construct rotation matrix from svd results */
        auto R = construct_rotation_matrix(svdResults[0], svdResults[1], svdResults[2]);

        /* combine with current rotation estimate using quaternions */
        Eigen::Quaterniond Rq(R);
        estimateQuat *= Rq;

        /* apply translation to estimate */
        estimateTranslation.x() += svdResults[3];
        estimateTranslation.y() += svdResults[4];
        estimateTranslation.z() += svdResults[5];

        /* apply error metric and increment iteration */
        error_value = averageDistanceSq;
        iteration++;
    }

    if (iteration >= ICP_MAX_ITERATIONS) {
        std::cout << "hit max iterations..., error: " << error_value << std::endl;
    } else {
        std::cout << "finished in iterations: " << iteration << std::endl;
    }

    /* fill poseEstimate with new data and return */
    poseEstimate.orientation.z = estimateQuat.z();
    poseEstimate.orientation.x = estimateQuat.x();
    poseEstimate.orientation.y = estimateQuat.y();
    poseEstimate.orientation.z = estimateQuat.z();
    poseEstimate.position.x = estimateTranslation.x();
    poseEstimate.position.y = estimateTranslation.y();
    poseEstimate.position.z = estimateTranslation.z();

    return poseEstimate;
}
