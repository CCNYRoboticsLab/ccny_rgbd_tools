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
#include "ccny_rgbd/structures/monocular_frame.h"

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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MonocularVisualOdometry(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MonocularVisualOdometry();
    /**
     * @brief Fitness function for RANSAC used to find the initial camera pose
     *
     * @param M the intrinsic 3x3 camera matrix
     * @param E the extrinsic 3x4 camera matrix (composed by rotation and translation parameters)
     * @param distance_threshold Fitness criterion for determining a good fit if distance (in pixels) between a nearest-neighbors pair is less than or equal to this threshold
     * @param min_inliers The minimum number of inliers as a criterion for determining a good fit of the hypothesis
     * @param sample_3D_points Random sample of 6 3D points from the sparse cloud  mode
     * @param feature_2D_points Detected 2D features in input image
     * @param inliers_3D_points Resulting inliers from the 3D point cloud model (pointer as member variable $model_ptr_$)
     * @param inliers_2D_points Resulting inliers from the set of 2D point features
     *
     * @return True if the fitness falls under certain threshold criteria of number of inliers
     */
    bool fitness(const cv::Mat M, const cv::Mat E, const int distance_threshold, const int min_inliers, const std::vector<cv::Point3d> &sample_3D_points, const std::vector<cv::Point2d> & feature_2D_points, std::vector<cv::Point3d> &inliers_3D_points, std::vector<cv::Point2d> & inliers_2D_points);

   // estimate the first camera pose
   cv::Mat estimateFirstPose(
     const cv::Mat& intrinsic_matrix, 
     const std::vector<cv::Point3d>& model,
     const std::vector<cv::Point2d>& image_2d_points,
     int min_inliers, 
     int max_iterations,
     int distance_threshold);

   void convertPointCloudModelPointsToVector(const PointCloudFeature::Ptr model);

   /**
    * @brief Fitness function for RANSAC used to find the initial camera pose
    *
    * @param model_3D the 3D point cloud model as a vector
    * @param features_2D the 2D keypoints (features) on the current frame
    * @param rvec the 3x1 rotation vector
    * @param tvec the 3x1 translation vector
    * @param corr_3D_points the vector of 3D points corresponding to the 2D points found
    * @param corr_2D_points the vector of 2D points correspondances to the 2D keypoints (features detetected on frame)
    *
    * @return The normalized accumulated distances (error) of the correspondences found
    */
   double getCorrespondences(const std::vector<cv::Point3d> &model_3D, const std::vector<cv::Point2d> &features_2D, const cv::Mat &rvec, const cv::Mat &tvec, std::vector<cv::Point3d> &corr_3D_points, std::vector<cv::Point2d> &corr_2D_points );

   // TODO: Roberto:
   void estimateMotion(const cv::Mat &E_prev, cv::Mat &E_new, const std::vector<cv::Point3d> &model, const std::vector<cv::Point2d> &features, int max_PnP_iterations = 10);

  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Publisher odom_publisher_;

//    boost::shared_ptr<image_transport::ImageTransport> rgb_it_;
    boost::shared_ptr<SynchronizerMonoVO> sync_;
    boost::shared_ptr<MonocularFrame> frame_;

    ImageSubFilter      sub_rgb_;
    CameraInfoSubFilter sub_info_;

    // **** parameters 

    std::string pcd_filename_;
    std::string fixed_frame_; 
    std::string base_frame_;

    std::string detector_type_;

    // **** variables
    boost::mutex::scoped_lock mutex_lock_; ///< Thread lock on subscribed input images
    bool initialized_;
    bool is_first_time_projecting_; ///< To indicate the first instance when the complete cloud model gets projected to the camera
    int  frame_count_;
    ros::Time init_time_;

    int min_inliers_;
    int max_iterations_;
    int distance_threshold_;
    int max_PnP_iterations_;

    tf::Transform b2c_;
    tf::Transform f2b_;

    FeatureDetector * feature_detector_;

    MotionEstimation * motion_estimation_;

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    PointCloudFeature::Ptr model_ptr_;
    std::vector<cv::Point3d> model_cloud_vector_;
    ros::Publisher pub_model_; ///< Publisher for the point cloud model (sparse map)

    bool publish_cloud_model_; ///< to indicate whether the model pointcloud will be published


    // Camera parameters
    double sensor_aperture_width_, sensor_aperture_height_; // TODO: find out values
    // Topic names:
    std::string topic_cam_info_;
    std::string topic_image_;

    // **** private functions
    void imageCallback(const ImageMsg::ConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
    void initParams();
    void publishTf(const std_msgs::Header& header);

    bool getBaseToCameraTf(const std_msgs::Header& header);
    void setFeatureDetector();
    bool readPointCloudFromPCDFile(); ///< Returns true if PCD file was read successfully.
    
    void testGetMatches();
    void getMatches (
        const cv::Mat& projected_points,
        const cv::Mat& detected_points,
        std::vector<int>& match_indices,
        std::vector<float>& match_distances);

};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MONO_VO
