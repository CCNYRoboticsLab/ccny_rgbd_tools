#include "ccny_rgbd/registration/motion_estimation_icp.h"

namespace ccny_rgbd
{

MotionEstimationICP::MotionEstimationICP(ros::NodeHandle nh, ros::NodeHandle nh_private):
  MotionEstimation(nh, nh_private)
{
  // **** init variables

  f2b_.setIdentity();
  last_keyframe_f2b_ = f2b_;

  // *** init params

  // TODO - change to dynamic
  kf_dist_eps_  = 0.15; 
  kf_angle_eps_ = 15.0 * M_PI / 180.0; 

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";

  int icp_max_iterations;
  double icp_tf_epsilon;
  double icp_max_corresp_dist;
  bool use_ransac_rejection;
  double ransac_inlier_threshold;
  int history_size;

  if (!nh_private_.getParam ("reg/ICP/max_iterations", icp_max_iterations))
    icp_max_iterations = 30;
  if (!nh_private_.getParam ("reg/ICP/tf_epsilon", icp_tf_epsilon))
    icp_tf_epsilon = 0.000001;
  if (!nh_private_.getParam ("reg/ICP/use_ransac_rejection", use_ransac_rejection))
    use_ransac_rejection = false;
  if (!nh_private_.getParam ("reg/ICP/max_corresp_dist", icp_max_corresp_dist))
    icp_max_corresp_dist = 0.15;
  if (!nh_private_.getParam ("reg/ICP/ransac_inlier_threshold", ransac_inlier_threshold))
    ransac_inlier_threshold = 0.10;
  if (!nh_private_.getParam ("reg/ICP/history_size", history_size))
    history_size = 5;

  reg_.setMaxIterations(icp_max_iterations);
  reg_.setTransformationEpsilon(icp_tf_epsilon);
  reg_.setMaxCorrDist(icp_max_corresp_dist);
  reg_.setUseRANSACRejection(use_ransac_rejection);
  reg_.setRANSACThreshold(ransac_inlier_threshold);

  feature_history_.setCapacity(history_size);
}

MotionEstimationICP::~MotionEstimationICP()
{

}

bool MotionEstimationICP::getMotionEstimationImpl(
  RGBDFrame& frame,
  const tf::Transform& prediction,
  tf::Transform& motion)
{
  bool result;

  PointCloudFeature::Ptr features_ptr =
    boost::shared_ptr<PointCloudFeature> (new PointCloudFeature());

  // account for prediction
  tf::Transform predicted_f2b = prediction * f2b_;

  // rotate into the fixed frame and account for prediction
  pcl::transformPointCloud(frame.features , *features_ptr, eigenFromTf(predicted_f2b * b2c_));
  features_ptr->header.frame_id = fixed_frame_;

  // **** build model ***************************************************

  PointCloudFeature::Ptr model_ptr =
    boost::shared_ptr<PointCloudFeature> (new PointCloudFeature());

  model_ptr->header.frame_id = fixed_frame_;

  // add in recent features
  feature_history_.getAll(*model_ptr);

  // add in last keyframe features
  *model_ptr += last_keyframe_features_;

  if (model_ptr->points.empty())
  {
    ROS_WARN("No points in model");
    last_keyframe_features_ = *features_ptr;  
    last_keyframe_f2b_ = f2b_;
    motion.setIdentity();
    result = false;
  }
  else
  {
    // **** icp ***********************************************************

    pcl::KdTreeFLANN<PointFeature> tree_data;
    pcl::KdTreeFLANN<PointFeature> tree_model;

    tree_data.setInputCloud(features_ptr);
    tree_model.setInputCloud(model_ptr);

    reg_.setDataCloud  (&*features_ptr);
    reg_.setModelCloud (&*model_ptr);

    reg_.setDataTree  (&tree_data);
    reg_.setModelTree (&tree_model);

    result = reg_.align();
  }

  if (result)
  { 
    tf::Transform correction = tfFromEigen(reg_.getFinalTransformation());
    motion = correction * prediction;
    constrainMotion(motion);

    f2b_ = motion * f2b_;

    // check if correction is big enough -> create keyframe
    if (tfGreaterThan(motion, kf_dist_eps_, kf_angle_eps_))
    {
      last_keyframe_features_ = *features_ptr;  
      last_keyframe_f2b_ = f2b_;
    }
  }
  else
  {
    feature_history_.reset();
    motion.setIdentity();
  }

  // transform features using the new estimate for the fixed frame
  pcl::transformPointCloud (frame.features , *features_ptr, eigenFromTf(f2b_* b2c_));
  features_ptr->header.frame_id = fixed_frame_;

  feature_history_.add(*features_ptr);

  return result;
}

} // namespace ccny_rgbd
