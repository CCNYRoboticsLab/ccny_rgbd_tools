#include "ccny_rgbd/registration/motion_estimation_icp_loop.h"

namespace ccny_rgbd
{

MotionEstimationICPLoop::MotionEstimationICPLoop(ros::NodeHandle nh, ros::NodeHandle nh_private):
  MotionEstimation(nh, nh_private)
{
  // **** init variables

  f2b_.setIdentity();

  // *** init params

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";

  int icp_max_iterations;
  double icp_tf_epsilon;
  double icp_max_corresp_dist;
  bool use_ransac_rejection;
  double ransac_inlier_threshold;

  if (!nh_private_.getParam ("reg/max_iterations", icp_max_iterations))
    icp_max_iterations = 30;
  if (!nh_private_.getParam ("reg/tf_epsilon", icp_tf_epsilon))
    icp_tf_epsilon = 0.000001;
  if (!nh_private_.getParam ("reg/use_ransac_rejection", use_ransac_rejection))
    use_ransac_rejection = false;
  if (!nh_private_.getParam ("reg/max_corresp_dist", icp_max_corresp_dist))
    icp_max_corresp_dist = 0.15;
  if (!nh_private_.getParam ("reg/ransac_inlier_threshold", ransac_inlier_threshold))
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
  features_publisher_ = nh_.advertise<PointCloudFeature>(
    "features", 1);
}

MotionEstimationICPLoop::~MotionEstimationICPLoop()
{

}

tf::Transform MotionEstimationICPLoop::getMotionEstimation(
  RGBDFrame& frame,
  const tf::Transform& prediction)
{
  tf::Transform motion;

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

    for (int i = 0; i < features_ptr->points.size(); ++i)
    {




    }
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

    bool result = reg_.align();
    
    if (result)
    { 
      tf::Transform correction = tfFromEigen(reg_.getFinalTransformation());

      f2b_ = correction * predicted_f2b;
      motion = correction * prediction;
    }
    else
    {
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

    float max_d = 0.05;
    float max_d_sq = max_d * max_d;
    float alpha = 0.01;

    for (unsigned int i = 0; i < features_ptr->points.size(); ++i)
    {
      PointFeature& p_f = features_ptr->points[i];
      tree_model.nearestKSearch(p_f, 1, indices, dist_sq);
      PointFeature& p_m = model_ptr_->points[indices[0]];

      if (dist_sq[0] < max_d_sq)
      {
        PointFeature p_updated;
        p_updated.x = alpha * p_f.x + (1.0-alpha) * p_m.x;
        p_updated.y = alpha * p_f.y + (1.0-alpha) * p_m.y;
        p_updated.z = alpha * p_f.z + (1.0-alpha) * p_m.z;

        model_ptr_->points[indices[0]] = p_updated;
      }
      else
      {
        // insert new feature
        model_ptr_->points.push_back(p_f);

        
      }
    }

    float dur = (ros::WallTime::now() - sm).toSec();
    printf("Model dur: %f ms\n", dur*1000.0); 

    model_ptr_->header.stamp = ros::Time::now(); // TODO - change to actual timestamp
    model_ptr_->width = model_ptr_->points.size();
    model_publisher_.publish(model_ptr_);
    printf("Model size: %d\n", model_ptr_->points.size()); 

    features_publisher_.publish(frame.features);
  }

  return motion;
}

} // namespace ccny_rgbd
