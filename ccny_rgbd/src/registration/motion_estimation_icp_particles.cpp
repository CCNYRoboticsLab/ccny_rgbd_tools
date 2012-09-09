#include "ccny_rgbd/registration/motion_estimation_icp_particles.h"

namespace ccny_rgbd
{

MotionEstimationICPParticles::MotionEstimationICPParticles(ros::NodeHandle nh, ros::NodeHandle nh_private):
  MotionEstimation(nh, nh_private)
{
  srand(time(NULL));

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

  model_publisher_ = nh_.advertise<PointCloudFeature>(
    "model", 1);
  features_publisher_ = nh_.advertise<PointCloudFeature>(
    "features", 1);

  n_particles_ = 50;

  models_.resize(n_particles_);

  printf("creating models...\n");
  for (unsigned int p_idx = 0; p_idx < n_particles_; ++p_idx)
  {
    //PointCloudFeature::Ptr model;
    //model.reset(new PointCloudFeature());
    //models_.push_back(model);
    models_[p_idx].reset(new PointCloudFeature());
    models_[p_idx]->header.frame_id = fixed_frame_;
  }
  printf("creating models done.\n");

  best_model_idx_ = 0;  
}

MotionEstimationICPParticles::~MotionEstimationICPParticles()
{

}

tf::Transform MotionEstimationICPParticles::getMotionEstimation(
  RGBDFrame& frame,
  const tf::Transform& prediction)
{
  tf::Transform motion;

  PointCloudFeature::Ptr& model_ptr_ = models_[best_model_idx_];

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

    for (int p_idx = 0; p_idx < models_.size(); ++p_idx)
    {
      PointCloudFeature::Ptr& model_ptr_p = models_[p_idx];
      *model_ptr_p += *features_ptr;
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
    tf::Transform correction;    

    if (result)
    { 
      correction = tfFromEigen(reg_.getFinalTransformation());
    }
    else
    {
      correction.setIdentity();
    }
  
    f2b_ = correction * predicted_f2b;
    motion = correction * prediction;

    // add to model

    ros::WallTime sm = ros::WallTime::now();
  
    for (int p_idx = 0; p_idx < models_.size(); ++p_idx)
    {
      PointCloudFeature::Ptr& model_ptr_p = models_[p_idx];

      pcl::KdTreeFLANN<PointFeature> tree_model_p;
      tree_model_p.setInputCloud(model_ptr_p);

      // randomize tf

      tf::Transform correction_p;
      if (p_idx> 0) addError(correction, correction_p);
      else  correction_p = correction;

      tf::Transform f2b_p = correction_p * predicted_f2b;     

      // transform features using the new estimate for the fixed frame
      pcl::transformPointCloud (frame.features , *features_ptr, eigenFromTf(f2b_p* b2c_));
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
        tree_model_p.nearestKSearch(p_f, 1, indices, dist_sq);
        PointFeature& p_m = model_ptr_p->points[indices[0]];

        if (dist_sq[0] < max_d_sq)
        {
          PointFeature p_updated;
          p_updated.x = alpha * p_f.x + (1.0-alpha) * p_m.x;
          p_updated.y = alpha * p_f.y + (1.0-alpha) * p_m.y;
          p_updated.z = alpha * p_f.z + (1.0-alpha) * p_m.z;

          model_ptr_p->points[indices[0]] = p_updated;
        }
        else
        {
          model_ptr_p->points.push_back(p_f);
        }
      }

      model_ptr_p->width = model_ptr_p->points.size();
      model_ptr_p->header.stamp = ros::Time::now(); // TODO - change to actual timestamp
    }

    // get best count

    int best_count = models_[0]->points.size();
    int best_count_idx = 0;
    for (int p_idx = 0; p_idx < models_.size(); ++p_idx)
    {
      PointCloudFeature::Ptr& model_ptr_p = models_[p_idx];
      int count = model_ptr_p->points.size();
      printf("%d ", count); 

      if (count < best_count)
      {
        best_count = count;
        best_count_idx = p_idx;
      }
    }
    
    printf(" [%d](%d)\n", best_count, best_count_idx);

    float dur = (ros::WallTime::now() - sm).toSec();
    printf("Model dur: %f ms\n", dur*1000.0); 

    model_publisher_.publish(models_[best_count_idx]);
    printf("Model size: %d\n", model_ptr_->points.size()); 

    features_publisher_.publish(frame.features);
  }

  return motion;
}

void MotionEstimationICPParticles::addError(const tf::Transform& tf_in, tf::Transform& tf_out)
{
  tf::Vector3 v = tf_in.getOrigin();
  float length = v.length();  

  float r = ((rand() % 1000)/1000.0);

  float scale = 1.0 + (r - 0.5)*0.05;

  tf_out = tf_in;
  tf_out.setOrigin(v * scale);
}

} // namespace ccny_rgbd
