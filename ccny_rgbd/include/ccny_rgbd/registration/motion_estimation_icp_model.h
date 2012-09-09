#ifndef CCNY_RGBD_MOTION_ESTIMATION_ICP_MODEL_H
#define CCNY_RGBD_MOTION_ESTIMATION_ICP_MODEL_H

#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>

#include "ccny_rgbd/registration/motion_estimation.h"
#include "ccny_rgbd/structures/feature_history.h"

namespace ccny_rgbd
{

class MotionEstimationICPModel: public MotionEstimation
{
  typedef ccny_rgbd::ICPKd<PointFeature, PointFeature> ICPReg;

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MotionEstimationICPModel(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MotionEstimationICPModel();

    bool getMotionEstimationImpl(
      RGBDFrame& frame,
      const tf::Transform& prediction,
      tf::Transform& motion);
  
  private:

    // **** ros

    ros::Publisher model_publisher_;

    // **** params

    std::string fixed_frame_; 
    std::string base_frame_;

    float max_association_dist_;
    float alpha_;

    // **** variables

    float max_association_dist_sq_;
    ICPReg reg_;
    PointCloudFeature::Ptr model_ptr_;   
    tf::Transform f2b_; // Fixed frame to Base (moving) frame
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MOTION_ESTIMATION_ICP_MODEL_H

