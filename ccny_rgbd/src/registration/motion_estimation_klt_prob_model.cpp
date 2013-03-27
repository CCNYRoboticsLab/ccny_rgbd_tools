#include "ccny_rgbd/registration/motion_estimation_klt_prob_model.h"

namespace ccny_rgbd
{

MotionEstimationKLTProbModel::MotionEstimationKLTProbModel(ros::NodeHandle nh, ros::NodeHandle nh_private):
  MotionEstimation(nh, nh_private),
  first_time_(true),
  model_idx_(0),
  model_size_(0)
{
  // **** init params  
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";

  if (!nh_private_.getParam ("reg/KLTProbModel/publish_model_cloud", publish_model_))
    publish_model_ = true;
  if (!nh_private_.getParam ("reg/KLTProbModel/publish_model_cov", publish_model_cov_))
    publish_model_cov_ = false;
  if (!nh_private_.getParam ("reg/KLTProbModel/max_model_size", max_model_size_))
    max_model_size_ = 3000;
  if (!nh_private_.getParam ("reg/KLTProbModel/max_assoc_dist_mah", max_assoc_dist_mah_))
    max_assoc_dist_mah_ = 10.0;
    
  /*
  if (!nh_private_.getParam ("reg/ICPProbModel/tf_epsilon_linear", tf_epsilon_linear_))
    tf_epsilon_linear_ = 1e-3; // 1 mm
  if (!nh_private_.getParam ("reg/ICPProbModel/tf_epsilon_angular", tf_epsilon_angular_))
    tf_epsilon_angular_ = 1.7e-2; // 1 deg
  if (!nh_private_.getParam ("reg/ICPProbModel/max_iterations", max_iterations_))
    max_iterations_ = 10;
  if (!nh_private_.getParam ("reg/ICPProbModel/min_correspondences", min_correspondences_))
    min_correspondences_ = 15;

  
  if (!nh_private_.getParam ("reg/ICPProbModel/max_corresp_dist_eucl", max_corresp_dist_eucl_))
    max_corresp_dist_eucl_ = 0.15;


  if (!nh_private_.getParam ("reg/ICPProbModel/n_nearest_neighbors", n_nearest_neighbors_))
    n_nearest_neighbors_ = 4;



  // **** variables


  // **** services

  save_service_ = nh_.advertiseService(
    "save_sparse_map", &MotionEstimationKLTProbModel::saveSrvCallback, this);
  load_service_ = nh_.advertiseService(
    "load_sparse_map", &MotionEstimationKLTProbModel::loadSrvCallback, this);
  */
  
  // derived params
  max_assoc_dist_mah_sq_ = max_assoc_dist_mah_ * max_assoc_dist_mah_;
  
  
  // **** publishers

  if (publish_model_)
  {
    model_publisher_ = nh_.advertise<PointCloudFeature>(
      "model", 1);
  }
  if (publish_model_cov_)
  {
    covariances_publisher_ = nh_.advertise<visualization_msgs::Marker>(
      "model_covariances", 1);
  }
  
  model_ptr_.reset(new PointCloudFeature());
  model_ptr_->header.frame_id = fixed_frame_;

  f2b_.setIdentity(); 
  I_.setIdentity();
}

MotionEstimationKLTProbModel::~MotionEstimationKLTProbModel()
{

}

void MotionEstimationKLTProbModel::addToModel(
  const Vector3f& data_mean,
  const Matrix3f& data_cov)
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
    
    model_size_++;
  }
  else // model_size_ == max_model_size_
  {   
    if (model_idx_ == max_model_size_)
      model_idx_ = 0;

    covariances_.at(model_idx_) = data_cov;
    means_.at(model_idx_) = data_mean;
    model_ptr_->at(model_idx_) = data_point;
  }

  model_idx_++;
}

bool MotionEstimationKLTProbModel::getMotionEstimationImpl(
  RGBDFrame& frame,
  const tf::Transform& prediction,
  tf::Transform& motion)
{ 
  bool result;
      
  // create input image from frame
  cv::Mat input_img(frame.rgb_img.size(), CV_8UC1);
  cvtColor(frame.rgb_img, input_img, CV_BGR2GRAY);
        
  // **** perform registration

  if (model_size_ == 0)
  {
    ROS_INFO("No points in model: initializing from features.");
    
    Vector3fVector data_means;
    Matrix3fVector data_covariances;

    // remove nans from distributinos
    removeInvalidDistributions(
      frame.kp_means, frame.kp_covariances, frame.kp_valid,
      data_means, data_covariances);
  
    // transform distributions to world frame
    transformDistributions(data_means, data_covariances, f2b_ * b2c_);
        
    motion.setIdentity();
    initializeModelFromData(data_means, data_covariances);
    
    result = true;
  }
  else
  {    
    // project model into a set of virtual 2D features 
    printf("projecting\n");       
    std::vector<cv::Point2f> virtual_points;   
    std::vector<int> virtual_point_indices;
    projectModelToFeaures(frame, virtual_points, virtual_point_indices);
                       
    // prepare KLT arguments
    printf("ktl\n");    
    std::vector<uchar> status;
    std::vector<float> err;      
    cv::TermCriteria criteria(
      cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
    int klt_max_level = 3;
    cv::Size klt_win_size(7,7);
   
    // newly detected points
    std::vector<cv::Point2f> image_points;
   
    calcOpticalFlowPyrLK(
      prev_img_, input_img, 
      virtual_points, image_points,
      status, err,
      klt_win_size, klt_max_level, criteria);    
        
    printf("virtual_points, image_points: %d, %d\n",
      (int)virtual_points.size(), (int)image_points.size());
        
    showTrackingImage(frame, image_points, virtual_points, status);     
        
    // filter out only the tracked ones
    std::vector<cv::Point2f> image_points_tracked;
    std::vector<cv::Point2f> virtual_points_tracked;  
    std::vector<int> virtual_point_tracked_indices; 
    
    for (unsigned int i = 0; i < image_points.size(); ++i)
    {
      // skip invalid data
      int u = image_points[i].x;
      int v = image_points[i].y;
      if (frame.depth_img.at<uint16_t>(v, u) == 0) continue;
    
      // skip untracked data
      if(status[i] != 1) continue;

      image_points_tracked.push_back(image_points[i]);
      virtual_points_tracked.push_back(virtual_points[i]);
      virtual_point_tracked_indices.push_back(virtual_point_indices[i]);
    }
    
    printf("virtual_points_tr, image_points_tr: %d, %d\n",
      (int)virtual_points_tracked.size(), (int)image_points_tracked.size());
    
    // save detected KLT points as keypoint if the frame
    printf("setting in frame\n");    
    frame.keypoints.clear();
    for (unsigned int i = 0; i < image_points_tracked.size(); ++i)
    {
      const cv::Point2f& point = image_points_tracked[i];
      cv::KeyPoint kp;
      kp.pt = point;
      frame.keypoints.push_back(kp);
    }
     
    // calculate distributions of KLT points
    printf("computing distributions\n");    
    frame.computeDistributions(999, 999);

    // filter out any points with invalid data  
    printf("filtering invalid\n");         
    Vector3fVector data_means;
    Matrix3fVector data_covariances;
    IntVector model_indices;

    assert(frame.kp_valid.size() == virtual_point_tracked_indices.size());

    for (unsigned int i = 0; i < frame.kp_valid.size(); ++i)
    {
      if (frame.kp_valid[i])
      {
        data_means.push_back(frame.kp_means[i]);
        data_covariances.push_back(frame.kp_covariances[i]);
        model_indices.push_back(virtual_point_tracked_indices[i]);
      }
    }    

    // rotate data distributions into the fixed frame     
    transformDistributions(data_means, data_covariances, f2b_ * b2c_);
   
    // extract a set of model means and covariances, indexed by model_indices               
    printf("extracing from model\n");    
    Vector3fVector model_means;  
    Matrix3fVector model_covariances;
    
    model_means.resize(model_indices.size());
    model_covariances.resize(model_indices.size());
   
    for (unsigned int i = 0; i < model_indices.size(); ++i)
    {      
      int idx = model_indices[i];
      model_means[i]       = means_[idx];
      model_covariances[i] = covariances_[idx];            
    }
 
    // align data to model using svd-ransac
    printf("aligning\n"); 
    IntVector inlier_indices;
    alignRansacSVD(data_means,  data_covariances, 
                   model_means, model_covariances, 
                   inlier_indices, motion);
                   
    f2b_ = motion * f2b_;
    
    // transform distributions by the motion (they are already in fixed frame)
    transformDistributions(data_means, data_covariances, motion);

    // update model: inserts new features and updates old ones with KF
    printf("kf updating\n"); 
    updateModelFromData(data_means, data_covariances, model_indices);
    
    result = true;
  }

  // rotate 
  prev_img_ = input_img.clone();

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

void MotionEstimationKLTProbModel::alignRansacSVD(
  const Vector3fVector& data_means,  
  const Matrix3fVector& data_covariances, 
  const Vector3fVector& model_means, 
  const Matrix3fVector& model_covariances, 
  IntVector& best_inlier_indices, 
  tf::Transform output_tf)
{
  int min_sample_size = 3;
  int max_ransac_iterations_ = 150;
  double sufficient_inlier_ratio = 0.6;
  double max_eucl_dist = 0.03;
  double max_eucl_dist_sq = max_eucl_dist * max_eucl_dist;
  bool reestimate = true;

  assert(data_means.size() == model_means.size());
  assert(data_covariances.size() == model_covariances.size());
  assert(data_means.size() == data_covariances.size());

  // create point clouds from the distributions
  PointCloudFeature data_cloud, model_cloud;
  pointCloudFromMeans(data_means, data_cloud);
  pointCloudFromMeans(model_means, model_cloud);
  
  // **** main RANSAC loop ****************************************
  TransformationEstimationSVD svd;
  unsigned int size = data_cloud.points.size();
 
  int best_n_inliers = 0;
  Eigen::Matrix4f transformation; // transformation used inside loop
  Eigen::Matrix4f best_transformation;
      
  for (int iteration = 0; iteration < max_ransac_iterations_; ++iteration)
  {   
    // generate random indices
    IntVector sample_idx;
    getRandomIndices(min_sample_size, size, sample_idx);
    
    // build initial inliers from random indices
    IntVector inlier_idx;

    for (unsigned int s_idx = 0; s_idx < sample_idx.size(); ++s_idx)
    {
      int m_idx = sample_idx[s_idx];
      inlier_idx.push_back(m_idx);
    } 
    
    // estimate transformation from minimum set of random samples
    svd.estimateRigidTransformation(
      data_cloud, inlier_idx,
      model_cloud, inlier_idx,
      transformation);

    // evaluate transformation fitness by checking distance to all points
    PointCloudFeature data_cloud_tf;
    pcl::transformPointCloud(data_cloud, data_cloud_tf, transformation);

    for (int m_idx = 0; m_idx < size; ++m_idx)
    {
      const PointFeature& p_d = data_cloud_tf[m_idx];
      const PointFeature& p_m = model_cloud[m_idx];

      float dist_sq = distEuclideanSq(p_d, p_m);
      
      if (dist_sq < max_eucl_dist_sq)
      {
        inlier_idx.push_back(m_idx);

        if (reestimate)
        {
          // reestimate transformation from all inliers
          svd.estimateRigidTransformation(
            data_cloud, inlier_idx,
            model_cloud, inlier_idx,
            transformation);
          pcl::transformPointCloud(data_cloud, data_cloud_tf, transformation);
        }
      }
    }
    
    // check if inliers are better than the best model so far
    int n_inliers = inlier_idx.size();

    if (n_inliers > best_n_inliers)
    {
      svd.estimateRigidTransformation(
        data_cloud, inlier_idx,
        model_cloud, inlier_idx,
        transformation);

      best_n_inliers = n_inliers;
      best_transformation = transformation;
      best_inlier_indices = inlier_idx;
    }

    // check if we reached ratio termination criteria
    double inlier_ratio = (double) n_inliers / (double) size;

    if (inlier_ratio > sufficient_inlier_ratio)
      break;
  }

  output_tf = tfFromEigen(best_transformation);
}

void MotionEstimationKLTProbModel::ransacSVD(
  const PointCloudFeature& data_cloud, 
  const PointCloudFeature& model_cloud,
  IntVector& best_inlier_indices,
  tf::Transform& output_tf)
{
  int min_sample_size = 3;
  int max_ransac_iterations_ = 150;
  double sufficient_inlier_ratio = 0.6;
  double max_eucl_dist = 0.03;
  double max_eucl_dist_sq = max_eucl_dist * max_eucl_dist;
  bool reestimate = true;

  TransformationEstimationSVD svd;

  // **** main RANSAC loop ****************************************
  
  int best_n_inliers = 0;
  Eigen::Matrix4f transformation; // transformation used inside loop
  Eigen::Matrix4f best_transformation;
    
  int size = data_cloud.points.size();
  
  for (int iteration = 0; iteration < max_ransac_iterations_; ++iteration)
  {   
    // generate random indices
    IntVector sample_idx;
    getRandomIndices(min_sample_size, size, sample_idx);
    
    // build initial inliers from random indices
    IntVector inlier_idx;

    for (unsigned int s_idx = 0; s_idx < sample_idx.size(); ++s_idx)
    {
      int m_idx = sample_idx[s_idx];
      inlier_idx.push_back(m_idx);
    } 
    
    // estimate transformation from minimum set of random samples
    svd.estimateRigidTransformation(
      data_cloud, inlier_idx,
      model_cloud, inlier_idx,
      transformation);

    // evaluate transformation fitness by checking distance to all points
    PointCloudFeature data_cloud_tf;
    pcl::transformPointCloud(data_cloud, data_cloud_tf, transformation);

    for (int m_idx = 0; m_idx < size; ++m_idx)
    {
      const PointFeature& p_d = data_cloud_tf[m_idx];
      const PointFeature& p_m = model_cloud[m_idx];

      float dist_sq = distEuclideanSq(p_d, p_m);
      
      if (dist_sq < max_eucl_dist_sq)
      {
        inlier_idx.push_back(m_idx);

        if (reestimate)
        {
          // reestimate transformation from all inliers
          svd.estimateRigidTransformation(
            data_cloud, inlier_idx,
            model_cloud, inlier_idx,
            transformation);
          pcl::transformPointCloud(data_cloud, data_cloud_tf, transformation);
        }
      }
    }
    
    // check if inliers are better than the best model so far
    int n_inliers = inlier_idx.size();

    if (n_inliers > best_n_inliers)
    {
      svd.estimateRigidTransformation(
        data_cloud, inlier_idx,
        model_cloud, inlier_idx,
        transformation);

      best_n_inliers = n_inliers;
      best_transformation = transformation;
      best_inlier_indices = inlier_idx;
    }

    // check if we reached ratio termination criteria
    double inlier_ratio = (double) n_inliers / (double) size;

    if (inlier_ratio > sufficient_inlier_ratio)
      break;
  }

  output_tf = tfFromEigen(best_transformation);
}

void MotionEstimationKLTProbModel::detectFeatures(
  const RGBDFrame& frame)
{
/*
  // create input image from frame
  cv::Mat input_img(frame.rgb_img.size(), CV_8UC1);
  cvtColor(frame.rgb_img, input_img, CV_BGR2GRAY);
  
  // project model into a set of virtaul 2D features        
  std::vector<cv::Point2f> virtual_points;   
  std::vector<int> virtual_point_indices;
  projectModelToFeaures(frame, virtual_points, virtual_point_indices);
          
  // prepare KLT arguments
  std::vector<uchar> status;
  std::vector<float> err;      
  cv::TermCriteria criteria(
    cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
  int klt_max_level = 3;
  cv::Size klt_win_size(7,7);
 
  // newly detected points
  std::vector<cv::Point2f> image_points;
 
  calcOpticalFlowPyrLK(
    prev_img_, input_img, 
    virtual_points, image_points,
    status, err,
    klt_win_size, klt_max_level, criteria);    
      
  showTrackingImage(frame, image_points, virtual_points, status);
  
  prev_points_ = image_points;
  */
}

void MotionEstimationKLTProbModel::getCorrespMahalanobis(
  const Vector3fVector& data_means,
  const Matrix3fVector& data_covariances,
  IntVector& data_indices,
  IntVector& model_indices)
{
  IntVector indices;
  FloatVector dists_sq;

  indices.resize(n_nearest_neighbors_);
  dists_sq.resize(n_nearest_neighbors_);

  for (unsigned int data_idx = 0; data_idx < data_means.size(); ++data_idx)
  {
    const Vector3f data_mean = data_means[data_idx];
    const Matrix3f data_cov  = data_covariances[data_idx];
    
    int mah_nn_idx;
    double mah_dist_sq;
    
    bool nn_result = getNNMahalanobis(
      data_mean, data_cov, mah_nn_idx, mah_dist_sq, indices, dists_sq);

    if (nn_result && mah_dist_sq < max_corresp_dist_mah_sq_)
    {
      data_indices.push_back(data_idx);
      model_indices.push_back(mah_nn_idx);
    }
  }  
}

void MotionEstimationKLTProbModel::getCorrespEuclidean(
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

bool MotionEstimationKLTProbModel::getNNEuclidean(
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

bool MotionEstimationKLTProbModel::getNNMahalanobis(
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
  
void MotionEstimationKLTProbModel::initializeModelFromData(
  const Vector3fVector& data_means,
  const Matrix3fVector& data_covariances)
{
  for (unsigned int idx = 0; idx < data_means.size(); ++idx)
  {
    const Vector3f& mean = data_means[idx];
    const Matrix3f& cov  = data_covariances[idx];     
    addToModel(mean, cov);
  }
}

void MotionEstimationKLTProbModel::updateModelFromData(
  const Vector3fVector& data_means,
  const Matrix3fVector& data_covariances,
  const IntVector& model_indices)
{
  assert(data_means.size() == data_covariances.size());
  assert(data_means.size() == model_indices.size());

  for (unsigned int idx = 0; idx < data_means.size(); ++idx)
  {
    // the data distribution
    const Vector3f& data_mean = data_means[idx];
    const Matrix3f& data_cov  = data_covariances[idx];
    
    // the model distribution
    int model_idx = model_indices[idx];
    Vector3f& model_mean = means_[model_idx];
    Matrix3f& model_cov  = covariances_[model_idx];
    
    // calculate ditance between them
    Vector3f diff_mat = model_mean - data_mean;
    Matrix3f sum_cov  = model_cov  + data_cov;
    Matrix3f sum_cov_inv = sum_cov.inverse();

    Eigen::Matrix<float,1,1> mah_mat = diff_mat.transpose() * sum_cov_inv * diff_mat;

    double mah_dist_sq = mah_mat(0,0);

    if (mah_dist_sq < max_assoc_dist_mah_sq_)
    {
      // **** KF update *********************************

      // predicted state
      const Vector3f& model_mean_pred = model_mean;
      const Matrix3f& model_cov_pred  = model_cov;
      
      // calculate measurement and cov residual
      Vector3f y = data_mean - model_mean_pred;
      Matrix3f S = data_cov + model_cov_pred;

      // calculate Kalman gain
      Matrix3f K = model_cov_pred * S.inverse();
      
      // updated state estimate (mean and cov)
      Vector3f model_mean_upd = model_mean_pred + K * y;
      Matrix3f model_cov_upd  = (I_ - K) * model_cov_pred;
      
      // update in model
      model_mean = model_mean_upd;
      model_cov  = model_cov_upd;

      PointFeature updated_point;
      updated_point.x = model_mean_upd(0,0);
      updated_point.y = model_mean_upd(1,0);
      updated_point.z = model_mean_upd(2,0);

      model_ptr_->points[model_idx] = updated_point;
    }
    else
    {
      //addToModel(data_mean, data_cov);
    }
  }
}

void MotionEstimationKLTProbModel::publishCovariances()
{
  // create markers
  visualization_msgs::Marker marker;
  marker.header.frame_id = fixed_frame_;
  marker.header.stamp = ros::Time::now(); //FIXME - correct timestamp
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
      double scale = sigma * 2.0;

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

bool MotionEstimationKLTProbModel::saveSrvCallback(
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

bool MotionEstimationKLTProbModel::loadSrvCallback(
  ccny_rgbd::Save::Request& request,
  ccny_rgbd::Save::Response& response)
{
  ROS_INFO("Loading model from %s...", request.filename.c_str());

  bool result = loadModel(request.filename);

  if (result)
    ROS_INFO("Successfully loaded model.");
  else
    ROS_ERROR("Failed to load model.");

  return result;
}

bool MotionEstimationKLTProbModel::saveModel(const std::string& filename)
{
// FIXME - load and save eigen
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

  return (result_pcd == 0); // TODO: also OpenCV result
}

bool MotionEstimationKLTProbModel::loadModel(const std::string& filename)
{
// FIXME - load and save eigen
/*
  // load OpenCV yml matrix
  std::string filename_yml = filename + ".yml";

  cv::FileStorage fs(filename_yml, cv::FileStorage::READ);
  fs["means"] >> means_;
  fs["covariances"] >> covariances_;
  fs["model_idx"] >> model_idx_;
  fs["model_size"] >> model_size_;
*/

  // load pcd
  std::string filename_pcd = filename + ".pcd";
  pcl::PCDReader reader;
  int result_pcd = reader.read<PointFeature>(filename_pcd, *model_ptr_);
  model_ptr_->header.frame_id = fixed_frame_;

  // update the model tree
  model_tree_.setInputCloud(model_ptr_);

  return (result_pcd == 0); // TODO: also OpenCV result
}

void MotionEstimationKLTProbModel::projectModelToFeaures(
  const RGBDFrame& frame,
  std::vector<cv::Point2f>& virtual_points, 
  std::vector<int>& virtual_point_indices)
{
  // get R and t matrices of current camera pose, in eigen form
  Matrix3f R;
  Vector3f t; 
  tfToEigenRt(f2b_ * b2c_, R, t);
  Matrix3f R_inv = R.inverse();

  //std::cout << "R" << std::endl << R << std::endl;
  //std::cout << "t" << std::endl << t << std::endl;

  // get intrinsic matrix of image, in Eigen form
  cv::Mat intr_cv = frame.model.intrinsicMatrix();
  Matrix3f intr;
  for (int u = 0; u < 3; ++u)
  for (int v = 0; v < 3; ++v)
    intr(v,u) = intr_cv.at<double>(v, u); 

  // iterate over model
  for (int idx = 0; idx < model_size_; ++idx)
  {
    // the point from the model, in the fixed frame
    const Vector3f& p = means_[idx];
    
    // transform into camera frame
    Vector3f p_cam = R_inv * (p - t);
    
    // project onto image
    Vector3f q = intr * p_cam;
    
    // calculate u and v coordinates
    double u = q(0,0) / q(2,0);
    double v = q(1,0) / q(2,0);
  
    // if inside image, add to output vectors
    if (u >= 0 && v < frame.rgb_img.cols &&
        v >= 0 && v < frame.rgb_img.rows)
    {
      cv::Point2f cv_q;
      cv_q.x = u;
      cv_q.y = v;
      
      virtual_points.push_back(cv_q);
      virtual_point_indices.push_back(idx);
    }
  }
  
  // show
  KeypointVector kpts_virtual;
  for (unsigned int i = 0; i < virtual_points.size(); ++i)
  {
    cv::KeyPoint kp_virtual;
    kp_virtual.pt = virtual_points[i];
    kpts_virtual.push_back(kp_virtual);
  }
  
  //printf("kpts_virtual.size() = %d\n", kpts_virtual.size());
    
  // show proj. image
  cv::Mat proj_img = frame.rgb_img.clone();
  cv::drawKeypoints(
    proj_img, kpts_virtual, proj_img, cv::Scalar(0,255,0));   
  cv::namedWindow("Proj", CV_WINDOW_NORMAL); 
  cv::imshow("Proj", proj_img);
}

void MotionEstimationKLTProbModel::showTrackingImage(
  const RGBDFrame& frame,
  const std::vector<cv::Point2f>& image_points, 
  const std::vector<cv::Point2f>& virtual_points, 
  const std::vector<uchar>& status)
{
  // transfer points2f to keypoints
  cv::Mat tracking_img = frame.rgb_img.clone();
  int n_tracked = 0;
  
  KeypointVector kpts_virtual;
  KeypointVector kpts_image;
  KeypointVector kpts_image_untracked;
  
  for (unsigned int i = 0; i < status.size(); ++i)
  {
    if(status[i] == 1)
    {
      cv::KeyPoint kp_virtual;
      kp_virtual.pt = virtual_points[i];
      kpts_virtual.push_back(kp_virtual);
      
      cv::KeyPoint kp_image;
      kp_image.pt = image_points[i];
      kpts_image.push_back(kp_image);
    
      //cv::line(
      //  tracking_img, virtual_points[i], image_points[i], 
      //  cv::Scalar(0,0,255), 1);
        
      n_tracked++;
    }
    else
    {
      cv::KeyPoint kp_image_untracked;
      kp_image_untracked.pt = image_points[i];
      kpts_image_untracked.push_back(kp_image_untracked);
    }
  }

  int n_untracked = status.size() - n_tracked;
  printf("tracked / untr: [%d / %d]\n", n_tracked, n_untracked);
    
  // draw tracked image keypoints, GREEN
  cv::drawKeypoints(
    tracking_img, kpts_image, tracking_img, cv::Scalar(0,255,0));             
  
  //// draw untracked image keypoints, RED
  //cv::drawKeypoints(
  //  tracking_img, kpts_image_untracked, tracking_img, cv::Scalar(0,0,255)); 
    
  // show tracking image
  cv::namedWindow("Tracking", CV_WINDOW_NORMAL);
  cv::imshow("Tracking", tracking_img);
  cv::waitKey(1);
}

} // namespace ccny_rgbd
