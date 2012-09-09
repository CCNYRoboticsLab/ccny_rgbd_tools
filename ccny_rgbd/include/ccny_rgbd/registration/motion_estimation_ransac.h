#ifndef CCNY_RGBD_MOTION_ESTIMATION_RANSAC_H
#define CCNY_RGBD_MOTION_ESTIMATION_RANSAC_H

#include <tf/transform_datatypes.h>

#include "ccny_rgbd/registration/motion_estimation.h"
#include "ccny_rgbd/structures/rgbd_keyframe.h"

namespace ccny_rgbd
{

class MotionEstimationRANSAC: public MotionEstimation
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MotionEstimationRANSAC(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MotionEstimationRANSAC();

    bool getMotionEstimationImpl(
      RGBDFrame& frame,
      const tf::Transform& prediction,
      tf::Transform& motion);
  
  private:

    // params

    std::string fixed_frame_; 
    std::string base_frame_;

    double kf_dist_eps_;
    double kf_angle_eps_;

    double matching_distance_;
    double eps_reproj_;
    double inlier_threshold_;
    bool icp_refine_;
  
    // variables

    bool have_prev_frame_;
    RGBDFrame prev_frame_;

    tf::Transform f2b_; // Fixed frame to Base (moving) frame

    tf::Transform     last_keyframe_f2b_;
    PointCloudFeature last_keyframe_features_;

    bool ransacMatchingOverlap(
      RGBDFrame& frame_src, 
      RGBDFrame& frame_dst, 
      tf::Transform& transform,
      PointCloudT::Ptr cloud_src = boost::shared_ptr<PointCloudT>(new PointCloudT()), 
      PointCloudT::Ptr cloud_dst = boost::shared_ptr<PointCloudT>(new PointCloudT()));

};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MOTION_ESTIMATION_RANSAC_H
