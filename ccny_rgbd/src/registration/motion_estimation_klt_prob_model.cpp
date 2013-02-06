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

  printf("creating ...\n");
  
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";

  /*
  if (!nh_private_.getParam ("reg/ICPProbModel/tf_epsilon_linear", tf_epsilon_linear_))
    tf_epsilon_linear_ = 1e-3; // 1 mm
  if (!nh_private_.getParam ("reg/ICPProbModel/tf_epsilon_angular", tf_epsilon_angular_))
    tf_epsilon_angular_ = 1.7e-2; // 1 deg
  if (!nh_private_.getParam ("reg/ICPProbModel/max_iterations", max_iterations_))
    max_iterations_ = 10;
  if (!nh_private_.getParam ("reg/ICPProbModel/min_correspondences", min_correspondences_))
    min_correspondences_ = 15;
  if (!nh_private_.getParam ("reg/ICPProbModel/max_model_size", max_model_size_))
    max_model_size_ = 3000;
  
  if (!nh_private_.getParam ("reg/ICPProbModel/max_corresp_dist_eucl", max_corresp_dist_eucl_))
    max_corresp_dist_eucl_ = 0.15;
  if (!nh_private_.getParam ("reg/ICPProbModel/max_corresp_dist_mah", max_corresp_dist_mah_))
    max_corresp_dist_mah_ = 10.0;
  if (!nh_private_.getParam ("reg/ICPProbModel/max_assoc_dist_mah", max_assoc_dist_mah_))
    max_assoc_dist_mah_ = 10.0;
  if (!nh_private_.getParam ("reg/ICPProbModel/n_nearest_neighbors", n_nearest_neighbors_))
    n_nearest_neighbors_ = 4;

  if (!nh_private_.getParam ("reg/ICPProbModel/publish_model", publish_model_))
    publish_model_ = false;
  if (!nh_private_.getParam ("reg/ICPProbModel/publish_model_cov", publish_model_cov_))
    publish_model_cov_ = true;

  // **** variables

  // derived params
  max_corresp_dist_eucl_sq_ = max_corresp_dist_eucl_ * max_corresp_dist_eucl_;
  max_corresp_dist_mah_sq_ =  max_corresp_dist_mah_ *  max_corresp_dist_mah_;
  max_assoc_dist_mah_sq_ = max_assoc_dist_mah_ * max_assoc_dist_mah_;
  
  model_ptr_.reset(new PointCloudFeature());
  model_ptr_->header.frame_id = fixed_frame_;

  f2b_.setIdentity(); 
  I_.setIdentity();

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

  // **** services

  save_service_ = nh_.advertiseService(
    "save_sparse_map", &MotionEstimationKLTProbModel::saveSrvCallback, this);
  load_service_ = nh_.advertiseService(
    "load_sparse_map", &MotionEstimationKLTProbModel::loadSrvCallback, this);
  */
  
  printf("done creating ...\n");
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
 cv::TermCriteria criteria(
    cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

  int klt_max_level = 3;
  cv::Size klt_win_size(21,21);
  
  cv::Mat input_img(frame.rgb_img.size(), CV_8UC1);
  cvtColor(frame.rgb_img, input_img, CV_BGR2GRAY);

  std::vector<cv::Point2f> points;
    
  if (first_time_)
  {
    printf("first time\n");
    cv::GoodFeaturesToTrackDetector gft_detector(300, 0.01, 5.0);

    cv::Mat mask(frame.depth_img.size(), CV_8UC1);
    frame.depth_img.convertTo(mask, CV_8U); 
       
    KeypointVector kp;
    
    gft_detector.detect(input_img, kp, mask);
    
    for (unsigned int i = 0; i < kp.size(); ++i)
      points.push_back(kp[i].pt);
    
    first_time_ = false;
  }
  else
  {
    std::vector<uchar> status;
    std::vector<float> err;
    
    printf("prev_kp_.size() = %d\n", (int)prev_points_.size());
    
    calcOpticalFlowPyrLK(
      prev_img_, input_img, 
      prev_points_, points,
      status, err,
      klt_win_size, klt_max_level, criteria);
    
    int n_tracked = 0;
    
    printf("prev: %d this: %d\n", (int)prev_points_.size(), (int)points.size());
    
    KeypointVector m_kpv;
    KeypointVector d_kpv;
    
    // transfer points2f to keyypoints
    frame.keypoints.clear();    
    for (unsigned int i = 0; i < status.size(); ++i)
    {
      if(status[i] == 1)
      {
        n_tracked++;
        
        cv::KeyPoint d_kp;
        d_kp.pt = points[i];
        d_kpv.push_back(d_kp);
        
        cv::KeyPoint m_kp;
        m_kp.pt = prev_points_[i];
        m_kpv.push_back(m_kp);
      }
    }
    
    printf("%d tracked\n", n_tracked);
    
    // show keypoints
    
    cv::namedWindow("Keypoints", CV_WINDOW_NORMAL);
    cv::Mat kp_img(frame.rgb_img.size(), CV_8UC1);
    cv::drawKeypoints(frame.rgb_img, d_kpv, kp_img, cv::Scalar(255,0,0));
            
    for (unsigned int i = 0; i < m_kpv.size(); ++i)
    {
      cv::line(kp_img, m_kpv[i].pt, d_kpv[i].pt, cv::Scalar(0,0,255), 1);
    }
    
    cv::imshow("Keypoints", kp_img);
    cv::waitKey(1);
  }
  
  prev_img_ = input_img.clone(); // TODO: clone needed?
  prev_points_ = points;

  return false;
  
  /*
  //TODO: currently ignores prediction

  bool result;
  Vector3fVector data_means;
  Matrix3fVector data_covariances;

  // remove nans from distributinos
  removeInvalidDistributions(
    frame.kp_means, frame.kp_covariances, frame.kp_valid,
    data_means, data_covariances);
  
  // transform distributions to world frame
  transformDistributions(data_means, data_covariances, f2b_ * b2c_);
       
  // **** perform registration

  if (model_size_ == 0)
  {
    ROS_INFO("No points in model: initializing from features.");
    motion.setIdentity();
    initializeModelFromData(data_means, data_covariances);
    result = true;
  }
  else
  {
    // align using icp 
    //result = alignICPEuclidean(data_means, motion);
    //result = alignICPMahalanobis(data_means, data_covariances, motion);

    if (!result) return false;

    constrainMotion(motion);
    f2b_ = motion * f2b_;
    
    // transform distributions to world frame
    transformDistributions(data_means, data_covariances, motion);

    // update model: inserts new features and updates old ones with KF
    updateModelFromData(data_means, data_covariances);
  }

  // update the model tree
  model_tree_.setInputCloud(model_ptr_);

  if (publish_model_)
  {
    // update model pointcloud and publish
    model_ptr_->header.stamp = frame.header.stamp;
    model_ptr_->width = model_ptr_->points.size();
    model_publisher_.publish(model_ptr_);
  }
  if (publish_model_cov_)
    publishCovariances();

  return result;
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
  const Matrix3fVector& data_covariances)
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
    }
    else
    {
      addToModel(data_mean, data_cov);
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

} // namespace ccny_rgbd
