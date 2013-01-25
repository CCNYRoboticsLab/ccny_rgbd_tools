/**
 *  @file motion_estimation_icp.cpp
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2013, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ccny_rgbd/registration/motion_estimation_icp.h"

namespace ccny_rgbd {

MotionEstimationICP::MotionEstimationICP(
  const ros::NodeHandle& nh, 
  const ros::NodeHandle& nh_private):
  MotionEstimation(nh, nh_private)
{
  // *** init params  
  
  int history_size;
  
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";

  if (!nh_private_.getParam ("reg/ICP/tf_epsilon_linear", tf_epsilon_linear_))
    tf_epsilon_linear_ = 1e-3; // 1 mm
  if (!nh_private_.getParam ("reg/ICP/tf_epsilon_angular", tf_epsilon_angular_))
    tf_epsilon_angular_ = 1.7e-2; // 1 deg
  if (!nh_private_.getParam ("reg/ICP/max_iterations", max_iterations_))
    max_iterations_ = 10;
  if (!nh_private_.getParam ("reg/ICP/min_correspondences", min_correspondences_))
    min_correspondences_ = 15; 
  if (!nh_private_.getParam ("reg/ICP/max_corresp_dist_eucl", max_corresp_dist_eucl_))
    max_corresp_dist_eucl_ = 0.15;
  if (!nh_private_.getParam ("reg/ICP/publish_model", publish_model_))
    publish_model_ = false;
  if (!nh_private_.getParam ("reg/ICP/history_size", history_size))
    history_size = 5;

  feature_history_.setCapacity(history_size);
  
  // derived
  
  max_corresp_dist_eucl_sq_ = max_corresp_dist_eucl_ * max_corresp_dist_eucl_;
  
  // **** init variables
  
  f2b_.setIdentity();
  
  model_ptr_.reset(new PointCloudFeature());
  model_ptr_->header.frame_id = fixed_frame_;
  
  // **** publishers

  if (publish_model_)
  {
    model_publisher_ = nh_.advertise<PointCloudFeature>(
      "model", 1);
  }
}

MotionEstimationICP::~MotionEstimationICP()
{

}

bool MotionEstimationICP::getMotionEstimationImpl(
  RGBDFrame& frame,
  const tf::Transform& prediction,
  tf::Transform& motion)
{
  /// @todo ignores prediction
  bool result;
 
  // **** create a data cloud from the means
  
  Vector3fVector data_means;
  removeInvalidMeans(frame.kp_means, frame.kp_valid, data_means);
  transformMeans(data_means, f2b_ * b2c_);
  
  // **** align ********************************************************
 
  if (model_ptr_->points.empty())
  {
    ROS_INFO("No points in model: initializing from features.");
    motion.setIdentity();
    result = true;
  }
  else
  {
    // align using icp 
    result = alignICPEuclidean(data_means, motion);
  }

  if (result)
  { 
    constrainMotion(motion);
    f2b_ = motion * f2b_;
  }
  else
  {
    feature_history_.reset();
    motion.setIdentity();
  }
  
  // **** update model  ***********************************************
 
  // transform features using the new estimate for the fixed frame
  PointCloudFeature features;
  frame.constructFeaturePointCloud(features);
  pcl::transformPointCloud(features , features, eigenFromTf(f2b_* b2c_));
  
  // aggregate in feature history
  features.header.frame_id = fixed_frame_;
  feature_history_.add(features);

   // update model
  model_ptr_->points.clear();
  feature_history_.getAll(*model_ptr_); 
  model_tree_.setInputCloud(model_ptr_);
  
  // **** publish visuals *********************************************
      
  if (publish_model_)
  {
    // update model pointcloud and publish
    model_ptr_->header.stamp = frame.header.stamp;
    model_ptr_->width = model_ptr_->points.size();
    model_publisher_.publish(model_ptr_);
  }
  
  return result;
}

bool MotionEstimationICP::alignICPEuclidean(
  const Vector3fVector& data_means,
  tf::Transform& correction)
{
  pcl::registration::TransformationEstimationSVD<PointFeature, PointFeature> svd;

  // create a point cloud from the means
  PointCloudFeature data_cloud;
  pointCloudFromMeans(data_means, data_cloud);

  // initialize the result transform
  Eigen::Matrix4f final_transformation; 
  final_transformation.setIdentity();
  
  for (int iteration = 0; iteration < max_iterations_; ++iteration)
  {    
    // get corespondences
    IntVector data_indices, model_indices;
    getCorrespEuclidean(data_cloud, data_indices, model_indices);
   
    if ((int)data_indices.size() <  min_correspondences_)
    {
      ROS_WARN("[ICP] Not enough correspondences (%d of %d minimum). Leacing ICP loop",
        (int)data_indices.size(),  min_correspondences_);
      return false;
    }

    // estimae transformation
    Eigen::Matrix4f transformation; 
    svd.estimateRigidTransformation (data_cloud, data_indices,
                                     *model_ptr_, model_indices,
                                     transformation);
    
    // rotate   
    pcl::transformPointCloud(data_cloud, data_cloud, transformation);
    
    // accumulate incremental tf
    final_transformation = transformation * final_transformation;

    // check for convergence
    double linear, angular;
    getTfDifference(
      tfFromEigen(transformation), linear, angular);
    if (linear  < tf_epsilon_linear_ &&
        angular < tf_epsilon_angular_)
    {
      //printf("(%f %f) conv. at [%d] leaving loop\n", 
      //  linear*1000.0, angular*10.0*180.0/3.14, iteration);
      break; 
    }
  }
  
  correction = tfFromEigen(final_transformation);
  return true;
}

void MotionEstimationICP::getCorrespEuclidean(
  const PointCloudFeature& data_cloud,
  IntVector& data_indices,
  IntVector& model_indices)
{
  for (unsigned int data_idx = 0; data_idx < data_cloud.size(); ++data_idx)
  {
    const PointFeature& data_point = data_cloud.points[data_idx];
    
    int eucl_nn_idx;
    double eucl_dist_sq;
    
    bool nn_result = getNNEuclidean(data_point, eucl_nn_idx, eucl_dist_sq);
    
    if (nn_result && eucl_dist_sq < max_corresp_dist_eucl_sq_)
    {
      data_indices.push_back(data_idx);
      model_indices.push_back(eucl_nn_idx);
    }
  }  
}

bool MotionEstimationICP::getNNEuclidean(
  const PointFeature& data_point,
  int& eucl_nn_idx, double& eucl_dist_sq)
{
  // find n Euclidean nearest neighbors
  IntVector indices;
  FloatVector dist_sq;
  
  indices.resize(1);
  dist_sq.resize(1);
  
  int n_retrieved = model_tree_.nearestKSearch(data_point, 1, indices, dist_sq);
  
  if (n_retrieved != 0)
  {
    eucl_nn_idx = indices[0];
    eucl_dist_sq = dist_sq[0];
    return true;
  }
  else return false;
}

} // namespace ccny_rgbd
