#include "ccny_rgbd/registration/motion_estimation_icp_model.h"

namespace ccny_rgbd
{

MotionEstimationICPModel::MotionEstimationICPModel(ros::NodeHandle nh, ros::NodeHandle nh_private):
  MotionEstimation(nh, nh_private)
{
  // **** init variables

  f2b_.setIdentity();

  // *** init params

  max_association_dist_ = 0.03;
  max_association_dist_sq_ = max_association_dist_ * max_association_dist_;
  alpha_ = 0.01;

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";

  int icp_max_iterations;
  double icp_tf_epsilon;
  double icp_max_corresp_dist;
  bool use_ransac_rejection;
  double ransac_inlier_threshold;

  if (!nh_private_.getParam ("reg/ICPModel/max_iterations", icp_max_iterations))
    icp_max_iterations = 30;
  if (!nh_private_.getParam ("reg/ICPModel/tf_epsilon", icp_tf_epsilon))
    icp_tf_epsilon = 0.000001;
  if (!nh_private_.getParam ("reg/ICPModel/use_ransac_rejection", use_ransac_rejection))
    use_ransac_rejection = false;
  if (!nh_private_.getParam ("reg/ICPModel/max_corresp_dist", icp_max_corresp_dist))
    icp_max_corresp_dist = 0.15;
  if (!nh_private_.getParam ("reg/ICPModel/ransac_inlier_threshold", ransac_inlier_threshold))
    ransac_inlier_threshold = 0.10;

  reg_.setMaxIterations(icp_max_iterations);
  reg_.setTransformationEpsilon(icp_tf_epsilon);
  reg_.setMaxCorrDist(icp_max_corresp_dist);
  reg_.setUseRANSACRejection(use_ransac_rejection);
  reg_.setRANSACThreshold(ransac_inlier_threshold);

  model_ptr_.reset(new PointCloudFeature());

  model_ptr_->header.frame_id = fixed_frame_;

  model_publisher_ = nh_.advertise<PointCloudFeature>(
    "model", 1);
}

MotionEstimationICPModel::~MotionEstimationICPModel()
{

}

bool MotionEstimationICPModel::getMotionEstimationImpl(
  RGBDFrame& frame,
  const tf::Transform& prediction,
  tf::Transform& motion)
{
  bool result;

  PointCloudFeature::Ptr features_ptr =
    boost::shared_ptr<PointCloudFeature> (new PointCloudFeature());

  // account for prediction
  // prediction is in the world frame
  tf::Transform predicted_f2b = prediction * f2b_;

  // rotate into the fixed frame and account for prediction
  pcl::transformPointCloud(frame.features , *features_ptr, eigenFromTf(predicted_f2b * b2c_));
  features_ptr->header.frame_id = fixed_frame_;

  // **** build model ***************************************************

  if (model_ptr_->points.empty())
  {
    ROS_WARN("No points in model");
    motion.setIdentity();

    // transform features and add to model
    pcl::transformPointCloud (frame.features , *features_ptr, eigenFromTf(f2b_* b2c_));
    features_ptr->header.frame_id = fixed_frame_;
    *model_ptr_ += *features_ptr;
  }
  else
  {
    // **** icp ***********************************************************

    pcl::KdTreeFLANN<PointFeature> tree_data;
    pcl::KdTreeFLANN<PointFeature> tree_model;

    tree_data.setInputCloud(features_ptr);
    tree_model.setInputCloud(model_ptr_);

    reg_.setDataCloud  (&*features_ptr);
    reg_.setModelCloud (&*model_ptr_);

    reg_.setDataTree  (&tree_data);
    reg_.setModelTree (&tree_model);

    result = reg_.align();
    
    if (result)
    { 
      tf::Transform correction = tfFromEigen(reg_.getFinalTransformation());

      motion = correction * prediction;
      constrainMotion(motion);

      f2b_ = motion * f2b_;
    }
    else
    {
      // TODO - clean this up
      f2b_ = predicted_f2b;
      motion = prediction;
    }
  
    // add to model

    ros::WallTime sm = ros::WallTime::now();

    // transform features using the new estimate for the fixed frame
    pcl::transformPointCloud (frame.features , *features_ptr, eigenFromTf(f2b_* b2c_));
    features_ptr->header.frame_id = fixed_frame_;

    std::vector<int> indices;
    std::vector<float> dist_sq;

    indices.resize(1);
    dist_sq.resize(1);

    for (unsigned int i = 0; i < features_ptr->points.size(); ++i)
    {
      PointFeature& p_f = features_ptr->points[i];
      tree_model.nearestKSearch(p_f, 1, indices, dist_sq);
      PointFeature& p_m = model_ptr_->points[indices[0]];

      // Associate using pt to point distance

      if (dist_sq[0] < max_association_dist_sq_)
      {
        PointFeature p_updated;
        p_updated.x = alpha_ * p_f.x + (1.0 - alpha_) * p_m.x;
        p_updated.y = alpha_ * p_f.y + (1.0 - alpha_) * p_m.y;
        p_updated.z = alpha_ * p_f.z + (1.0 - alpha_) * p_m.z;

        model_ptr_->points[indices[0]] = p_updated;
      }
      else
      {
        model_ptr_->points.push_back(p_f);
      }
    }
  }

  model_ptr_->header.stamp = ros::Time::now(); // TODO - change to actual timestamp
  model_ptr_->width = model_ptr_->points.size();
  model_publisher_.publish(model_ptr_);
  printf("Model size: %d\n", (int)model_ptr_->points.size()); 

  return result;
}

} // namespace ccny_rgbd
