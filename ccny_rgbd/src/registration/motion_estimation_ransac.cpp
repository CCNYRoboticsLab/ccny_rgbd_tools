/**
 *  @file motion_estimation_icp_prob_model.cpp
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

#include "ccny_rgbd/registration/motion_estimation_ransac.h"

namespace ccny_rgbd {

MotionEstimationRansac::MotionEstimationRansac(
  const ros::NodeHandle& nh, 
  const ros::NodeHandle& nh_private):
  MotionEstimation(nh, nh_private),
  model_idx_(0),
  model_size_(0)
{
  // **** init params

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";

  if (!nh_private_.getParam ("reg/ICPProbModel/tf_epsilon_linear", tf_epsilon_linear_))
    tf_epsilon_linear_ = 1e-4; // 1 mm
  if (!nh_private_.getParam ("reg/ICPProbModel/tf_epsilon_angular", tf_epsilon_angular_))
    tf_epsilon_angular_ = 1.7e-3; // 1 deg
  if (!nh_private_.getParam ("reg/ICPProbModel/max_iterations", max_iterations_))
    max_iterations_ = 10;
  if (!nh_private_.getParam ("reg/ICPProbModel/min_correspondences", min_correspondences_))
    min_correspondences_ = 15;
  if (!nh_private_.getParam ("reg/ICPProbModel/max_model_size", max_model_size_))
    max_model_size_ = 3000;
  
  if (!nh_private_.getParam ("reg/ICPProbModel/max_corresp_dist_eucl", max_corresp_dist_eucl_))
    max_corresp_dist_eucl_ = 0.15;
  if (!nh_private_.getParam ("reg/ICPProbModel/max_assoc_dist_mah", max_assoc_dist_mah_))
    max_assoc_dist_mah_ = 10.0;
  if (!nh_private_.getParam ("reg/ICPProbModel/n_nearest_neighbors", n_nearest_neighbors_))
    n_nearest_neighbors_ = 4;

  if (!nh_private_.getParam ("reg/ICPProbModel/publish_model_cloud", publish_model_))
    publish_model_ = false;
  if (!nh_private_.getParam ("reg/ICPProbModel/publish_model_covariances", publish_model_cov_))
    publish_model_cov_ = false;

  // **** variables

  // derived params
  max_corresp_dist_eucl_sq_ = max_corresp_dist_eucl_ * max_corresp_dist_eucl_;
  max_assoc_dist_mah_sq_ = max_assoc_dist_mah_ * max_assoc_dist_mah_;
  
  model_ptr_.reset(new PointCloudFeature());
  model_ptr_->header.frame_id = fixed_frame_;

  f2b_.setIdentity(); 
  I_.setIdentity();

  // **** publishers

  if (publish_model_)
  {
    model_publisher_ = nh_.advertise<PointCloudFeature>(
      "model/cloud", 1);
  }
  if (publish_model_cov_)
  {
    covariances_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      "model/covariances", 1);
  }

  // **** services

  save_service_ = nh_.advertiseService(
    "save_sparse_map", &MotionEstimationRansac::saveSrvCallback, this);
}

MotionEstimationRansac::~MotionEstimationRansac()
{

}

void MotionEstimationRansac::addToModel(
  const Vector3f& data_mean,
  const Matrix3f& data_cov,
  const cv::Mat& data_descriptor)
{
  // **** create a PCL point

  PointFeature data_point;
  data_point.x = data_mean(0,0);
  data_point.y = data_mean(1,0);
  data_point.z = data_mean(2,0);

  if (model_size_ < max_model_size_)
  { 
    covariances_.push_back(data_cov);
    means_.push_back(data_mean);
    model_ptr_->push_back(data_point);
    model_descriptors_.push_back(data_descriptor);
    
    model_size_++;
  }
  else // model_size_ == max_model_size_
  {   
    if (model_idx_ == max_model_size_)
      model_idx_ = 0;

    covariances_.at(model_idx_) = data_cov;
    means_.at(model_idx_) = data_mean;
    model_ptr_->at(model_idx_) = data_point;
    model_descriptors_.at(model_idx_) = data_descriptor;
  }

  model_idx_++;
}

bool MotionEstimationRansac::getMotionEstimationImpl(
  RGBDFrame& frame,
  const tf::Transform& prediction,
  tf::Transform& motion)
{
  /// @todo: currently ignores prediction

  bool result;
  Vector3fVector data_means;
  Matrix3fVector data_covariances;
  MatVector data_descriptors;

  // remove nans from distributinos
  removeInvalidDistributions(
    frame.kp_means, frame.kp_covariances, frame.descriptors, frame.kp_valid,
    data_means, data_covariances, data_descriptors);
  
  // transform distributions to world frame
  transformDistributions(data_means, data_covariances, f2b_ * b2c_);
       
  // **** perform registration

  if (model_size_ == 0)
  {
    ROS_INFO("No points in model: initializing from features.");
    motion.setIdentity();
    initializeModelFromData(data_means, data_covariances, data_descriptors);
    result = true;
  }
  else
  {
    result = alignRANSAC(data_means, data_descriptors, motion);
    //result = alignPnPRANSAC(frame, motion);

    if (!result) return false;

    constrainMotion(motion);
    f2b_ = motion * f2b_;
    
    // transform distributions to world frame
    transformDistributions(data_means, data_covariances, motion);

    // update model: inserts new features and updates old ones with KF
    updateModelFromData(data_means, data_covariances, data_descriptors);
  }

  // update the model tree
  model_tree_.setInputCloud(model_ptr_);

  // update the model timestamp and auxiliary info
  model_ptr_->header.stamp = frame.header.stamp;
  model_ptr_->width = model_ptr_->points.size();

  // publish data for visualization
  if (publish_model_)
    model_publisher_.publish(model_ptr_);
  if (publish_model_cov_)
    publishCovariances();

  return result;
}


bool MotionEstimationRansac::alignPnPRANSAC(
  const RGBDFrame& frame,
  tf::Transform& correction)
{
  // params
  double max_desc_dist = 40.0;
  int max_ransac_iterations = 500;
  
  double max_reprojection_error = 3.0;
  double min_inliers_count = 150;
    
  // ************************************************************
  // remove invalid
  MatVector data_descriptors;
  KeypointVector data_keypoints;
  for (unsigned int i = 0; i < frame.kp_valid.size(); ++i)
  {
    // copy keypoint
    cv::KeyPoint keypoint = frame.keypoints[i];
    cv::Mat descriptor = frame.descriptors.rowRange(i, i+1);
    
    if (frame.kp_valid[i])
    {
      data_keypoints.push_back(keypoint);
      data_descriptors.push_back(descriptor);     
    }
  }
  
  // ************************************************************
  // find all matches
  cv::BFMatcher matcher(cv::NORM_HAMMING);          // for ORB
  TransformationEstimationSVD svd;

  std::vector<cv::DMatch> all_matches;
      
  cv::Mat data_descriptors_mat(data_descriptors.size(), data_descriptors[0].cols, CV_8U);
  
  for (unsigned int j = 0; j < data_descriptors.size(); ++j)
  for (unsigned int i = 0; i < data_descriptors[0].cols; ++i)
  {
    data_descriptors_mat.at<uint8_t>(j, i) = data_descriptors[j].at<uint8_t>(0, i);
  }
  
  cv::Mat model_descriptors_mat(model_descriptors_.size(), model_descriptors_[0].cols, CV_8U);

  for (unsigned int j = 0; j < model_descriptors_.size(); ++j)
  for (unsigned int i = 0; i < model_descriptors_[0].cols; ++i)
  {
    model_descriptors_mat.at<uint8_t>(j, i) = model_descriptors_[j].at<uint8_t>(0, i);
  }
  
  matcher.match(data_descriptors_mat, model_descriptors_mat, all_matches);
  
  // ************************************************************
  // remove bad matches - too far away in descriptor space,
  //                    - nan, too far, or cov. too big
  std::vector<cv::DMatch> candidate_matches;
  for (unsigned int m_idx = 0; m_idx < all_matches.size(); ++m_idx)
  {
    const cv::DMatch& match = all_matches[m_idx];

    if (match.distance < max_desc_dist)
    {
      candidate_matches.push_back(all_matches[m_idx]);
    }
  }

  int size = candidate_matches.size();
    
  /// @fixme
  printf("size: %d\n", size);
  if (size < 3)
  {
    ROS_ERROR("size");
    return false;
  }
  
  // ************************************************************
  // build 3D features for SVD

  std::vector<cv::Point3f> points_3d;
  std::vector<cv::Point2f> points_2d;
  
  points_3d.resize(size);
  points_2d.resize(size);

  for (int m_idx = 0; m_idx < size; ++m_idx)
  {
    const cv::DMatch& match = candidate_matches[m_idx];
    int idx_data  = match.queryIdx;
    int idx_model = match.trainIdx; 
    
    points_2d[m_idx] = data_keypoints[idx_data].pt;

    cv::Point3f& p_model = points_3d[m_idx];
    p_model.x = means_[idx_model](0,0);
    p_model.y = means_[idx_model](1,0);
    p_model.z = means_[idx_model](2,0);
  }
   
  // ************************************************************
  // perform PnP
  
  tf::Transform f2c = f2b_ * b2c_;
  tf::Transform f2c_inv = f2c.inverse();
  tf::Matrix3x3 R = f2c_inv.getBasis();
  tf::Vector3 pose = f2c_inv.getOrigin();

  cv::Mat rmat(3, 3, CV_64F);
  for (int v = 0; v < 3; ++v)
  for (int u = 0; u < 3; ++u)
    rmat.at<double>(v, u) = R[v][u];

  cv::Mat rvec;
  cv::Rodrigues(rmat, rvec);
  
  cv::Mat tvec(3, 1, CV_64F);
  tvec.at<double>(0,0) = pose.getX();
  tvec.at<double>(1,0) = pose.getY();
  tvec.at<double>(2,0) = pose.getZ();
   
  //std::cout << "tvec in: " << tvec << std::endl;
  //std::cout << "rmat in: " << std::endl << rmat << std::endl;
  
  std::vector<int> inliers;
  
  solvePnPRansac(
    points_3d, points_2d, 
    frame.model.intrinsicMatrix(), cv::Mat(),
    rvec, tvec, true, 
    max_ransac_iterations, max_reprojection_error, min_inliers_count, 
    inliers, CV_ITERATIVE);  
 
  cv::Rodrigues(rvec, rmat);
  
  // ************************************************************
  // update pose
  
  std::cout << "tvec out: " << tvec << std::endl;
  std::cout << "rmat out: " << std::endl << rmat << std::endl;
  
  for (int v = 0; v < 3; ++v)
  for (int u = 0; u < 3; ++u)
    R[v][u] = rmat.at<double>(v, u);
  
  pose.setX(tvec.at<double>(0,0));
  pose.setY(tvec.at<double>(1,0));
  pose.setZ(tvec.at<double>(2,0));
  
  tf::Transform f2c_new_inv;
  f2c_new_inv.setOrigin(pose);
  f2c_new_inv.setBasis(R);
  tf::Transform f2c_new = f2c_new_inv.inverse();
   
  printf("INLIERS: %d\n", inliers.size());
  
  correction = f2c_new * f2c_inv;
  
  return true;
}

bool MotionEstimationRansac::alignRANSAC(
  const Vector3fVector& data_means,
  const MatVector data_descriptors,
  tf::Transform& correction)
{
  // params
  double max_desc_dist = 40.0;
  int max_ransac_iterations_ = 500;
  int min_sample_size = 3;
  double sufficient_inlier_ratio = 0.90;
  
  double max_eucl_dist = 0.03;
  double max_eucl_dist_sq = max_eucl_dist * max_eucl_dist;
  
  // ************************************************************
  // find all matches
  cv::BFMatcher matcher(cv::NORM_HAMMING);          // for ORB
  TransformationEstimationSVD svd;

  std::vector<cv::DMatch> all_matches;
      
  cv::Mat data_descriptors_mat(data_descriptors.size(), data_descriptors[0].cols, CV_8U);
  
  for (int j = 0; j < data_descriptors.size(); ++j)
  for (int i = 0; i < data_descriptors[0].cols; ++i)
  {
    data_descriptors_mat.at<uint8_t>(j, i) = data_descriptors[j].at<uint8_t>(0, i);
  }
  
  cv::Mat model_descriptors_mat(model_descriptors_.size(), model_descriptors_[0].cols, CV_8U);

  for (int j = 0; j < model_descriptors_.size(); ++j)
  for (int i = 0; i < model_descriptors_[0].cols; ++i)
  {
    model_descriptors_mat.at<uint8_t>(j, i) = model_descriptors_[j].at<uint8_t>(0, i);
  }
  
  matcher.match(data_descriptors_mat, model_descriptors_mat, all_matches);
  
  // ************************************************************
  // remove bad matches - too far away in descriptor space,
  //                    - nan, too far, or cov. too big
  std::vector<cv::DMatch> candidate_matches;
  for (unsigned int m_idx = 0; m_idx < all_matches.size(); ++m_idx)
  {
    const cv::DMatch& match = all_matches[m_idx];

    if (match.distance < max_desc_dist)
    {
      candidate_matches.push_back(all_matches[m_idx]);
    }
  }

  int size = candidate_matches.size();
    
  /// @fixme
  printf("size: %d\n", size);
  if (size < 3) return false;
  
  // ************************************************************
  // build 3D features for SVD

  PointCloudFeature points_data, points_model;

  points_data.resize(size);
  points_model.resize(size);

  for (int m_idx = 0; m_idx < size; ++m_idx)
  {
    const cv::DMatch& match = candidate_matches[m_idx];
    int idx_data  = match.queryIdx;
    int idx_model = match.trainIdx; 

    PointFeature& p_data = points_data[m_idx];
    p_data.x = data_means[idx_data](0,0);
    p_data.y = data_means[idx_data](1,0);
    p_data.z = data_means[idx_data](2,0);

    PointFeature& p_model = points_model[m_idx];
    p_model.x = means_[idx_model](0,0);
    p_model.y = means_[idx_model](1,0);
    p_model.z = means_[idx_model](2,0);
  }
  
  // ************************************************************
  // main RANSAC loop 

  int best_n_inliers = 0;
  std::vector<cv::DMatch> best_inlier_matches;
  Eigen::Matrix4f transformation; // transformation used inside loop
  Eigen::Matrix4f best_transformation;
  
  for (int iteration = 0; iteration < max_ransac_iterations_; ++iteration)
  {
    // generate random indices
    IntVector sample_idx;
    getRandomIndices(min_sample_size, size, sample_idx);

    // build initial inliers from random indices
    IntVector inlier_idx;
    std::vector<cv::DMatch> inlier_matches;

    for (unsigned int s_idx = 0; s_idx < sample_idx.size(); ++s_idx)
    {
      int m_idx = sample_idx[s_idx];
      inlier_idx.push_back(m_idx);
      inlier_matches.push_back(candidate_matches[m_idx]);
    } 

    // estimate transformation from minimum set of random samples
    svd.estimateRigidTransformation(
      points_data, inlier_idx,
      points_model, inlier_idx,
      transformation);

    // evaluate transformation fitness by checking distance to all points
    PointCloudFeature points_data_tf;
    pcl::transformPointCloud(points_data, points_data_tf, transformation);

    for (int m_idx = 0; m_idx < size; ++m_idx)
    {
      // euclidean distance
      const PointFeature& p_m = points_model[m_idx];
      const PointFeature& p_d = points_data_tf[m_idx];
      float dist_sq = distEuclideanSq(p_m, p_d);
      
      // todo: mahalanobis distance
      
      if (dist_sq < max_eucl_dist_sq)
      {
        inlier_idx.push_back(m_idx);
        inlier_matches.push_back(candidate_matches[m_idx]);

        // reestimate transformation from all inliers
        svd.estimateRigidTransformation(
          points_data, inlier_idx,
          points_model, inlier_idx,
          transformation);
        pcl::transformPointCloud(points_data, points_data_tf, transformation);
      }
    }

    // check if inliers are better than the best model so far
    int n_inliers = inlier_idx.size();

    if (n_inliers > best_n_inliers)
    {
      svd.estimateRigidTransformation(
        points_data, inlier_idx,
        points_model, inlier_idx,
        transformation);

      best_n_inliers = n_inliers;
      best_transformation = transformation;
      best_inlier_matches = inlier_matches;
    }

    // check if we reached ratio termination criteria
    double inlier_ratio = (double) n_inliers / (double) size;

    if (inlier_ratio > sufficient_inlier_ratio)
      break;
  }
    
  correction = tfFromEigen(best_transformation);
  
  return true; 
}

bool MotionEstimationRansac::alignICPEuclidean(
  const Vector3fVector& data_means,
  tf::Transform& correction)
{
  TransformationEstimationSVD svd;

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

void MotionEstimationRansac::getCorrespEuclidean(
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

bool MotionEstimationRansac::getNNEuclidean(
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

bool MotionEstimationRansac::getNNMahalanobis(
  const Vector3f& data_mean, const Matrix3f& data_cov,
  int& mah_nn_idx, double& mah_dist_sq,
  IntVector& indices, FloatVector& dists_sq)
{
  PointFeature p_data;
  p_data.x = data_mean(0,0);
  p_data.y = data_mean(1,0);
  p_data.z = data_mean(2,0);

  int n_retrieved = model_tree_.nearestKSearch(p_data, n_nearest_neighbors_, indices, dists_sq);

  // iterate over Euclidean NNs to find Mah. NN
  double best_mah_dist_sq = 0;
  int best_mah_nn_idx = -1;
  //int best_i = 0; // optionally print this to check how far in we found the best one
  for (int i = 0; i < n_retrieved; i++)
  {
    int nn_idx = indices[i];
   
    const Vector3f& model_mean = means_[nn_idx];
    const Matrix3f& model_cov  = covariances_[nn_idx];

    Vector3f diff_mat = model_mean - data_mean;
    Matrix3f sum_cov = model_cov + data_cov;
    Matrix3f sum_cov_inv = sum_cov.inverse();

    Eigen::Matrix<float,1,1> mah_mat = diff_mat.transpose() * sum_cov_inv * diff_mat;

    double mah_dist_sq = mah_mat(0,0);
  
    if (best_mah_nn_idx == -1 || mah_dist_sq < best_mah_dist_sq)
    {
      best_mah_dist_sq = mah_dist_sq;
      best_mah_nn_idx  = nn_idx;
      //best_i = i;
    }
  }

  if (best_mah_nn_idx != -1)
  {  
    //if (best_i != 0) printf("BEST NEIGHBOR WAS #%d\n", best_i);
    mah_dist_sq = best_mah_dist_sq;
    mah_nn_idx  = best_mah_nn_idx;
    return true;
  }
  else return false;
}
  
void MotionEstimationRansac::initializeModelFromData(
  const Vector3fVector& data_means,
  const Matrix3fVector& data_covariances,
  const MatVector& data_descriptors)
{
  for (unsigned int idx = 0; idx < data_means.size(); ++idx)
  {
    const Vector3f& mean = data_means[idx];
    const Matrix3f& cov  = data_covariances[idx];     
    const cv::Mat&  desc = data_descriptors[idx];
    addToModel(mean, cov, desc);
  }
}

void MotionEstimationRansac::updateModelFromData(
  const Vector3fVector& data_means,
  const Matrix3fVector& data_covariances,
  const MatVector& data_descriptors)
{
  // pre-allocate search vectors
  IntVector indices;
  FloatVector dists_sq;
  indices.resize(n_nearest_neighbors_);
  dists_sq.resize(n_nearest_neighbors_);

  for (unsigned int idx = 0; idx < data_means.size(); ++idx)
  {
    const Vector3f& data_mean = data_means[idx];
    const Matrix3f& data_cov  = data_covariances[idx];
    const cv::Mat&  data_desc = data_descriptors[idx];
    
    // find nearest neighbor in model 
    double mah_dist_sq;
    int mah_nn_idx;   
    bool nn_result = getNNMahalanobis(
      data_mean, data_cov, mah_nn_idx, mah_dist_sq, indices, dists_sq);
  
    if (nn_result && mah_dist_sq < max_assoc_dist_mah_sq_)
    {
      // **** KF update *********************************

      // predicted state
      const Vector3f& model_mean_pred = means_[mah_nn_idx];
      const Matrix3f& model_cov_pred  = covariances_[mah_nn_idx];
      
      // calculate measurement and cov residual
      Vector3f y = data_mean - model_mean_pred;
      Matrix3f S = data_cov + model_cov_pred;

      // calculate Kalman gain
      Matrix3f K = model_cov_pred * S.inverse();
      
      // updated state estimate (mean and cov)
      Vector3f model_mean_upd = model_mean_pred + K * y;
      Matrix3f model_cov_upd  = (I_ - K) * model_cov_pred;
      
      // update in model
      means_[mah_nn_idx] = model_mean_upd;
      covariances_[mah_nn_idx] = model_cov_upd;

      PointFeature updated_point;
      updated_point.x = model_mean_upd(0,0);
      updated_point.y = model_mean_upd(1,0);
      updated_point.z = model_mean_upd(2,0);

      model_ptr_->points[mah_nn_idx] = updated_point;
      
      // update descriptor
      cv::Mat& model_desc = model_descriptors_[mah_nn_idx];
      model_desc = data_desc;
    }
    else
    {
      addToModel(data_mean, data_cov, data_desc);
    }
  }
}

void MotionEstimationRansac::publishCovariances()
{
  // create markers
  visualization_msgs::Marker marker;
  marker.header.frame_id = fixed_frame_;
  marker.header.stamp = model_ptr_->header.stamp;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.0025;
  marker.action = visualization_msgs::Marker::ADD;
  marker.ns = "model_covariances";
  marker.id = 0;
  marker.lifetime = ros::Duration();

  for (unsigned int i = 0; i < model_ptr_->points.size(); ++i)
  {  
    // compute eigenvectors
    cv::Mat evl(1, 3, CV_64F);
    cv::Mat evt(3, 3, CV_64F);

    const Matrix3f& cov_eigen = covariances_[i];

    cv::Mat cov(3,3,CV_64F);
    for(int j = 0; j < 3; ++j)  
    for(int i = 0; i < 3; ++i)
      cov.at<double>(j,i) = cov_eigen(j,i);

    cv::eigen(cov, evl, evt);

    double mx = model_ptr_->points[i].x;
    double my = model_ptr_->points[i].y;
    double mz = model_ptr_->points[i].z;

    for (int e = 0; e < 3; ++e)
    {
      geometry_msgs::Point a;
      geometry_msgs::Point b;

      double sigma = sqrt(std::abs(evl.at<double>(0,e)));
      double scale = sigma * 3.0;

      tf::Vector3 evt_tf(evt.at<double>(e,0), 
                         evt.at<double>(e,1), 
                         evt.at<double>(e,2));

      a.x = mx + evt_tf.getX() * scale;
      a.y = my + evt_tf.getY() * scale;
      a.z = mz + evt_tf.getZ() * scale;
   
      b.x = mx - evt_tf.getX() * scale;
      b.y = my - evt_tf.getY() * scale;
      b.z = mz - evt_tf.getZ() * scale;

      marker.points.push_back(a);
      marker.points.push_back(b);
    }
  }

  covariances_publisher_.publish(marker);
}

bool MotionEstimationRansac::saveSrvCallback(
  ccny_rgbd::Save::Request& request,
  ccny_rgbd::Save::Response& response)
{
  ROS_INFO("Saving model to %s", request.filename.c_str());

  bool result = saveModel(request.filename);

  if (result)
    ROS_INFO("Successfully saved model.");
  else
    ROS_ERROR("Failed to save model.");

  return result;
}

bool MotionEstimationRansac::saveModel(const std::string& filename)
{
  /// @todo also save Eigen means and covariances 
/*
  // save as OpenCV yml matrix
  std::string filename_yml = filename + ".yml";

  cv::FileStorage fs(filename_yml, cv::FileStorage::WRITE);
  fs << "means"       << means_;
  fs << "covariances" << covariances_; 
  fs << "model_idx"   << model_idx_;  
  fs << "model_size"  << model_size_;    
*/   

  // save as pcd
  std::string filename_pcd = filename + ".pcd";
  pcl::PCDWriter writer;
  int result_pcd = writer.writeBinary<PointFeature>(filename_pcd, *model_ptr_);

  return (result_pcd == 0); 
}

// produces k random numbers in the range [0, n).
// Monte-Carlo based random sampling
void MotionEstimationRansac::getRandomIndices(
  int k, int n, IntVector& output)
{
  while ((int)output.size() < k)
  {
    int random_number = rand() % n;
    bool duplicate = false;    

    for (unsigned int i = 0; i < output.size(); ++i)
    {
      if (output[i] == random_number)
      {
        duplicate = true;
        break;
      }
    }

    if (!duplicate)
      output.push_back(random_number);
  }
}

} // namespace ccny_rgbd
