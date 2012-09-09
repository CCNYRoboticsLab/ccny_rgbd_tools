#ifndef CCNY_RGBD_MOTION_ESTIMATION_RANSAC_MODEL_H
#define CCNY_RGBD_MOTION_ESTIMATION_RANSAC_MODEL_H

#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>

#include "ccny_rgbd/registration/motion_estimation.h"

namespace ccny_rgbd
{

class MotionEstimationRANSACModel: public MotionEstimation
{
  typedef ccny_rgbd::ICPKd<PointFeature, PointFeature> ICPReg;

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MotionEstimationRANSACModel(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MotionEstimationRANSACModel();

    tf::Transform getMotionEstimation(
      RGBDFrame& frame,
      const tf::Transform& prediction);
  
  private:

    // **** ros

    ros::Publisher model_publisher_;

    // **** params

    std::string fixed_frame_; 
    std::string base_frame_;

    //float max_association_dist_;
    //float alpha_;

    // **** variables

    cv::FlannBasedMatcher matcher_;  
    cv::BFMatcher bf_matcher_;  

    //float max_association_dist_sq_;
    //ICPReg reg_;
    PointCloudFeature::Ptr model_ptr_;   

    std::vector<cv::KeyPoint> model_keypoints_;
    cv::Mat model_descriptors_;

    cv::Mat descriptors;

    tf::Transform f2b_; // Fixed frame to Base (moving) frame
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MOTION_ESTIMATION_RANSAC_MODEL_H
