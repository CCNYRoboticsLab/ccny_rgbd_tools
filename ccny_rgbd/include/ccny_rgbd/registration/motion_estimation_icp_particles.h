#ifndef CCNY_RGBD_MOTION_ESTIMATION_ICP_PARTICLES_H
#define CCNY_RGBD_MOTION_ESTIMATION_ICP_PARTICLES_H

#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>

#include "ccny_rgbd/registration/motion_estimation.h"
#include "ccny_rgbd/structures/feature_history.h"

namespace ccny_rgbd
{

class MotionEstimationICPParticles: public MotionEstimation
{
  typedef ccny_rgbd::ICPKd<PointFeature, PointFeature> ICPReg;
  typedef std::vector<
      PointCloudFeature::Ptr, 
      Eigen::aligned_allocator<PointCloudFeature::Ptr> 
    > PointCloudFeaturePtrVector;

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MotionEstimationICPParticles(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MotionEstimationICPParticles();

    tf::Transform getMotionEstimation(
      RGBDFrame& frame,
      const tf::Transform& prediction);
  
  private:

    // params
    std::string fixed_frame_; 
    std::string base_frame_;

    ros::Publisher model_publisher_;
    ros::Publisher features_publisher_;

    int n_particles_;

    // variables

    ICPReg reg_;

    unsigned int best_model_idx_;

    PointCloudFeaturePtrVector models_; 
    
    tf::Transform f2b_; // Fixed frame to Base (moving) frame

    void addError(const tf::Transform& tf_in, tf::Transform& tf_out);

};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MOTION_ESTIMATION_ICP_PARTICLES_H
