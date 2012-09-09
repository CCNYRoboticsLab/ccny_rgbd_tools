#ifndef CCNY_RGBD_RGBD_FRAME_H
#define CCNY_RGBD_RGBD_FRAME_H

#include <vector>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <pcl_ros/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include "ccny_rgbd/rgbd_util.h"
#include "ccny_rgbd/registration/icp_kd.h"

namespace ccny_rgbd
{

class RGBDFrame
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RGBDFrame();
    
    RGBDFrame(const sensor_msgs::ImageConstPtr& rgb_msg,
              const sensor_msgs::ImageConstPtr& depth_msg,
              const sensor_msgs::CameraInfoConstPtr& info_msg);
    
    std_msgs::Header header;

    PointCloudFeature features;
      
    std::vector<cv::KeyPoint> keypoints;

    std::vector<bool>    kp_valid;         // z is valid
    std::vector<cv::Mat> kp_mean;          // 1x3 mat of 3D location
    std::vector<cv::Mat> kp_covariance;    // 3x3 mat of covariance

    cv::Mat descriptors;

    bool keypoints_computed;
    bool descriptors_computed;

    cv::Mat * getRGBImage()   const { return &cv_ptr_rgb_->image; }
    cv::Mat * getDepthImage() const { return &cv_ptr_depth_->image; }

    void computeDistributions();
    void constructFeatureCloud(float max_range, bool filter=true);

    static void constructCloudsFromInliers(
      const std::vector<cv::DMatch>& inlier_matches,
      const RGBDFrame& frame_src, 
      const RGBDFrame& frame_dst, 
      PointCloudT::Ptr& cloud_src,
      PointCloudT::Ptr& cloud_dst);

    static bool ransacMatchingOverlap(
      RGBDFrame& frame_src, RGBDFrame& frame_dst, 
      tf::Transform& transform, float matching_distance, 
      float eps_reproj, float inlier_threshold,
      PointCloudT::Ptr cloud_src = boost::shared_ptr<PointCloudT>(new PointCloudT()), 
      PointCloudT::Ptr cloud_dst = boost::shared_ptr<PointCloudT>(new PointCloudT()));
  
    image_geometry::PinholeCameraModel model_;

  protected:

    cv_bridge::CvImagePtr cv_ptr_rgb_;
    cv_bridge::CvImagePtr cv_ptr_depth_;

    //double getGMMUncertaintyZ(int u, int v);
    double getVarZ(double z);
    double getStdDevZ(double z);

    void getGaussianDistribution(int u, int v, double& z_mean, double& z_var);
    void getGaussianMixtureDistribution(int u, int v, double& z_mean, double& z_var);

};

} //namespace ccny_rgbd


#endif // CCNY_RGBD_RGBD_FRAME_H
