#ifndef CCNY_RGBD_MONO_VO
#define CCNY_RGBD_MONO_VO

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//#include <opencv2/calib3d/calib3d.hpp>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/rgbd_util.h"
#include "ccny_rgbd/structures/rgbd_frame.h"

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/features/orb_detector.h"
#include "ccny_rgbd/features/surf_detector.h"
#include "ccny_rgbd/features/gft_detector.h"

#include "ccny_rgbd/registration/motion_estimation.h"
#include "ccny_rgbd/registration/motion_estimation_icp.h"
#include "ccny_rgbd/registration/motion_estimation_icp_prob_model.h"

namespace ccny_rgbd
{

using namespace message_filters::sync_policies;

class MonocularVisualOdometry
{
  typedef nav_msgs::Odometry OdomMsg;

  public:

    MonocularVisualOdometry(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MonocularVisualOdometry();

  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Publisher odom_publisher_;

//    boost::shared_ptr<image_transport::ImageTransport> rgb_it_;
    boost::shared_ptr<SynchronizerMonoVO> sync_;

    ImageSubFilter      sub_rgb_;
    CameraInfoSubFilter sub_info_;

    // **** parameters 

    std::string pcd_filename_;
    std::string fixed_frame_; 
    std::string base_frame_;

    std::string detector_type_;

    // **** variables
    boost::mutex mutex_lock_; ///< Thread lock on subscribed input images
    boost::mutex mutex_;
    bool initialized_;
    int  frame_count_;
    ros::Time init_time_;

    tf::Transform b2c_;
    tf::Transform f2b_;

    FeatureDetector * feature_detector_;

    MotionEstimation * motion_estimation_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

    // Camera parameters
    cv::Mat K_; ///< intrinsic parameter matrix
    cv::Mat R_; ///< rotation matrix
    cv::Mat T_; ///< translation vector
    cv::Mat rvec_; //< rotation vector
    cv::Mat tvec_; ///< translation vector
    cv::Mat dist_coeffs_; ///< distortion coefficients
    double sensor_aperture_width_, sensor_aperture_height_; // TODO: find out values
    cv::Point2d principal_point_; ///< Location of principal/optical center point in perspective camera (obtained from calibration)

    // **** private functions

    void imageCallback(const ImageMsg::ConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
    void processCameraInfo(const sensor_msgs::CameraInfoConstPtr &cam_info_ptr);
    void initParams();
    void publishTf(const std_msgs::Header& header);

    bool getBaseToCameraTf(const std_msgs::Header& header);
    void setFeatureDetector();
    bool readPointCloudFromPCDFile(); ///< Returns true if PCD file was read successfully.
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MONO_VO
