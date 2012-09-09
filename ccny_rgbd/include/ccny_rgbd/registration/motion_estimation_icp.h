#ifndef CCNY_RGBD_MOTION_ESTIMATION_ICP_H
#define CCNY_RGBD_MOTION_ESTIMATION_ICP_H

#include <tf/transform_datatypes.h>

#include "ccny_rgbd/registration/motion_estimation.h"
#include "ccny_rgbd/structures/feature_history.h"

namespace ccny_rgbd
{

class MotionEstimationICP: public MotionEstimation
{
  typedef ccny_rgbd::ICPKd<PointFeature, PointFeature> ICPReg;

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MotionEstimationICP(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MotionEstimationICP();

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

    // variables

    ICPReg reg_;

    FeatureHistory<PointFeature> feature_history_;

    tf::Transform f2b_; // Fixed frame to Base (moving) frame

    tf::Transform     last_keyframe_f2b_;
    PointCloudFeature last_keyframe_features_;
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MOTION_ESTIMATION_ICP_H
