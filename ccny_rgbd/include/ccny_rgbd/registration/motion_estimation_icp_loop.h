#ifndef CCNY_RGBD_MOTION_ESTIMATION_ICP_LOOP_H
#define CCNY_RGBD_MOTION_ESTIMATION_ICP_LOOP_H

#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>

#include "ccny_rgbd/registration/motion_estimation.h"
#include "ccny_rgbd/structures/feature_history.h"

namespace ccny_rgbd
{

class MotionEstimationICPLoop: public MotionEstimation
{
  typedef ccny_rgbd::ICPKd<PointFeature, PointFeature> ICPReg;

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MotionEstimationICPLoop(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MotionEstimationICPLoop();

    tf::Transform getMotionEstimation(
      RGBDFrame& frame,
      const tf::Transform& prediction);
  
  private:

    // params

    std::string fixed_frame_; 
    std::string base_frame_;

    ros::Publisher model_publisher_;
    ros::Publisher features_publisher_;

    // variables

    ICPReg reg_;

    PointCloudFeature::Ptr model_ptr_; 

    std::vector<cv::KeyPoint> keypoints_;
    cv::Mat descriptors_;   
    
    tf::Transform f2b_; // Fixed frame to Base (moving) frame
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MOTION_ESTIMATION_ICP_LOOP_H
