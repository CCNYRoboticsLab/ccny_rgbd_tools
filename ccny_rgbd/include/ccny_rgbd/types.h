#ifndef CCNY_RGBD_TYPES_H
#define CCNY_RGBD_TYPES_H

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace ccny_rgbd {

// **** typedefs *********************************************

typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Vector3f Vector3f;

typedef std::vector<int>             IntVector;
typedef std::vector<float>           FloatVector;
typedef std::vector<bool>            BoolVector;
typedef std::vector<cv::Point2f>     Point2fVector;
typedef std::vector<cv::Point3f>     Point3fVector;
typedef std::vector<Eigen::Matrix3f> Matrix3fVector;
typedef std::vector<Eigen::Vector3f> Vector3fVector;
typedef std::vector<cv::KeyPoint>    KeypointVector;

typedef pcl::PointXYZRGB              PointT;
typedef pcl::PointCloud<PointT>       PointCloudT;

typedef pcl::PointXYZ                 PointFeature;
typedef pcl::PointCloud<PointFeature> PointCloudFeature;

typedef sensor_msgs::Image            ImageMsg;
typedef sensor_msgs::CameraInfo       CameraInfoMsg;

typedef image_transport::SubscriberFilter ImageSubFilter;
typedef message_filters::Subscriber<CameraInfoMsg> CameraInfoSubFilter;
typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, CameraInfoMsg> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
typedef message_filters::sync_policies::ApproximateTime<ImageMsg, CameraInfoMsg> SyncPolicyMonoVO;
typedef message_filters::Synchronizer<SyncPolicyMonoVO> SynchronizerMonoVO;

} // namespace ccny_rgbd

#endif // CCNY_RGBD_TYPES_H
