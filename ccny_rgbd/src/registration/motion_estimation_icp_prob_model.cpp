#include "ccny_rgbd/registration/motion_estimation_icp_prob_model.h"

namespace ccny_rgbd
{

MotionEstimationICPProbModel::MotionEstimationICPProbModel(ros::NodeHandle nh, ros::NodeHandle nh_private):
  MotionEstimation(nh, nh_private),
  model_idx_(0),
  model_size_(0)
{
  // **** init variables

  f2b_.setIdentity();
  
  I_ = cv::Mat(3, 3, CV_64F);
  cv::setIdentity(I_);
  
  // **** init params

  max_association_dist_ = 0.03;
  max_association_dist_sq_ = max_association_dist_ * max_association_dist_;

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";

  int icp_max_iterations;
  double icp_tf_epsilon;
  double icp_max_corresp_dist;

  if (!nh_private_.getParam ("reg/ICPProbModel/max_iterations", icp_max_iterations))
    icp_max_iterations = 30;
  if (!nh_private_.getParam ("reg/ICPProbModel/tf_epsilon", icp_tf_epsilon))
    icp_tf_epsilon = 0.000001;
  if (!nh_private_.getParam ("reg/ICPProbModel/max_corresp_dist", icp_max_corresp_dist))
    icp_max_corresp_dist = 0.15;

  if (!nh_private_.getParam ("reg/ICPProbModel/max_model_size", max_model_size_))
    max_model_size_ = 3000;
  if (!nh_private_.getParam ("reg/ICPProbModel/max_association_dist", max_association_dist_mah_))
    max_association_dist_mah_ = 7.0;
  if (!nh_private_.getParam ("reg/ICPProbModel/n_nearest_neighbors", n_nearest_neighbors_))
    n_nearest_neighbors_ = 1;
  if (!nh_private_.getParam ("reg/ICPProbModel/alpha", alpha_))
    alpha_ = 0.5;

  reg_.setMaxIterations(icp_max_iterations);
  reg_.setTransformationEpsilon(icp_tf_epsilon);
  reg_.setMaxCorrDist(icp_max_corresp_dist);

  model_ptr_.reset(new PointCloudFeature());
  tree_model_.reset(new KdTree());

  model_ptr_->header.frame_id = fixed_frame_;

  model_publisher_ = nh_.advertise<PointCloudFeature>(
    "model", 1);
  covariances_publisher_ = nh_.advertise<visualization_msgs::Marker>(
    "model_covariances", 1);

  // **** services

  save_service_ = nh_.advertiseService(
    "save_sparse_map", &MotionEstimationICPProbModel::saveSrvCallback, this);
  load_service_ = nh_.advertiseService(
    "load_sparse_map", &MotionEstimationICPProbModel::loadSrvCallback, this);
}

MotionEstimationICPProbModel::~MotionEstimationICPProbModel()
{

}

void MotionEstimationICPProbModel::addToModel(
  const cv::Mat& feature_mean,
  const cv::Mat& feature_cov)
{
  // **** create a PCL point

  PointFeature feature_point;
  feature_point.x = feature_mean.at<double>(0,0);
  feature_point.y = feature_mean.at<double>(1,0);
  feature_point.z = feature_mean.at<double>(2,0);

  if (model_size_ < max_model_size_)
  { 
    covariances_.push_back(feature_cov);
    means_.push_back(feature_mean);
    model_ptr_->push_back(feature_point);
    
    model_size_++;
  }
  else // model_size_ == max_model_size_
  {   
    if (model_idx_ == max_model_size_)
      model_idx_ = 0;

    covariances_.at(model_idx_) = feature_cov;
    means_.at(model_idx_) = feature_mean;
    model_ptr_->at(model_idx_) = feature_point;
  }

  model_idx_++;
}

bool MotionEstimationICPProbModel::getMotionEstimationImpl(
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

  // **** perform registration

  if (model_size_ == 0)
  {
    ROS_INFO("No points in model: initializing from features.");
    motion = prediction;
    constrainMotion(motion);
    f2b_ = motion * f2b_;
    initializeModelFromFrame(frame);
    result = true;
  }
  else
  {
    // **** icp 
    reg_.setDataCloud (features_ptr); 
    result = reg_.align();
    
    if (result)
    { 
      tf::Transform correction = tfFromEigen(reg_.getFinalTransformation());
      motion = correction * prediction;
    }
    else
    {
      motion = prediction;
    }

    constrainMotion(motion);
    f2b_ = motion * f2b_;
    
    // update model: inserts new features and updates old ones with KF
    updateModelFromFrame(frame);
  }

  // update the model tree
  tree_model_->setInputCloud(model_ptr_);
  reg_.setModelCloud(model_ptr_);
  reg_.setModelTree(tree_model_);
  
  // update model pointcloud and publish
  model_ptr_->header.stamp = frame.header.stamp;
  model_ptr_->width = model_ptr_->points.size();
  model_publisher_.publish(model_ptr_);

  publishCovariances();

  return result;
}

void MotionEstimationICPProbModel::getNNMahalanobis(
  const KdTree& model_tree,
  const cv::Mat& f_mean, const cv::Mat& f_cov,
  double& mah_dist, int& mah_nn_idx)
{
  // precalculate inverse matrix
  cv::Mat f_cov_inv;
  cv::invert(f_cov, f_cov_inv);

  // find n Euclidean nearest neighbors
  std::vector<int> indices;
  std::vector<float> dist_sq;
  
  indices.resize(n_nearest_neighbors_);
  dist_sq.resize(n_nearest_neighbors_);

  PointFeature p_f;
  p_f.x = f_mean.at<double>(0,0);
  p_f.y = f_mean.at<double>(1,0);
  p_f.z = f_mean.at<double>(2,0);

  model_tree.nearestKSearch(p_f, 1, indices, dist_sq);

  // iterate over Euclidean NNs to find Mah. NN

  // TODO - clean this up
  double best_mah_dist = 9999999;
  int best_mah_nn_idx = -1;
  int best_i; // optionally print this to check how far in we found the best one
  for (int i = 0; i < n_nearest_neighbors_; i++)
  {
    int nn_idx = indices[i];
   
    const cv::Mat& m_mean = means_[nn_idx];
    const cv::Mat& m_cov  = covariances_[nn_idx];

    cv::Mat diff_mat = m_mean - f_mean;
    cv::Mat sum_cov = m_cov + f_cov;
    cv::Mat sum_cov_inv;
    cv::invert(sum_cov, sum_cov_inv);

    cv::Mat mah_mat = diff_mat.t() * sum_cov_inv * diff_mat;

    double mah_dist = sqrt(mah_mat.at<double>(0,0));

    if (mah_dist < best_mah_dist)
    {
      best_mah_dist   = mah_dist;
      best_mah_nn_idx = nn_idx;
      best_i = i;
    }
  }

  if (best_i != 0) printf("BEST NEIGHBOR WAS #%d\n", best_i);
  mah_dist   = best_mah_dist;
  mah_nn_idx = best_mah_nn_idx;
}

void MotionEstimationICPProbModel::initializeModelFromFrame(
  const RGBDFrame& frame)
{
  // calculate rotation and translation matrices
  cv::Mat t, R;
  transformToRotationCV(f2b_ * b2c_, t, R);
  
  for (unsigned int kp_idx = 0; kp_idx < frame.kp_covariance.size(); ++kp_idx)
  {
    if (!frame.kp_valid[kp_idx]) continue;

    const cv::Mat& feature_mean = frame.kp_mean[kp_idx];
    const cv::Mat& feature_cov  = frame.kp_covariance[kp_idx];

    cv::Mat feature_mean_tf = R * feature_mean + t;
    cv::Mat feature_cov_tf  = R * feature_cov * R.t();  

    addToModel(feature_mean_tf, feature_cov_tf);
  }
}

void MotionEstimationICPProbModel::updateModelFromFrame(
  const RGBDFrame& frame)
{
  // decompose transform
  cv::Mat t, R;
  transformToRotationCV(f2b_ * b2c_, t, R);
  
  for (unsigned int kp_idx = 0; kp_idx < frame.keypoints.size(); ++kp_idx)
  {
    // skip invalid
    if (!frame.kp_valid[kp_idx]) continue;  

    const cv::Mat& feature_mean = frame.kp_mean[kp_idx];
    const cv::Mat& feature_cov = frame.kp_covariance[kp_idx];

    // rotate to fixed frame
    cv::Mat feature_mean_tf = R * feature_mean + t;
    cv::Mat feature_cov_tf  = R * feature_cov * R.t();  

    // find nearest neighbor in model 
    double mah_dist;
    int mah_nn_idx;   
    getNNMahalanobis(*tree_model_, feature_mean_tf, feature_cov_tf, 
                     mah_dist, mah_nn_idx);
  
    if (mah_dist < max_association_dist_mah_)
    {
      // **** KF update *********************************

      // predicted state
      const cv::Mat& model_mean_pred = means_[mah_nn_idx];
      const cv::Mat& model_cov_pred  = covariances_[mah_nn_idx];
      
      // calculate measurement and cov residual
      cv::Mat y = feature_mean_tf - model_mean_pred;
      cv::Mat S = feature_cov_tf + model_cov_pred;

      // calculate Kalman gain
      cv::Mat K = model_cov_pred * S.inv();
      
      // updated state estimate (mean and cov)
      cv::Mat model_mean_upd = model_mean_pred + K * y;
      cv::Mat model_cov_upd  = (I_ - K) * model_cov_pred;
      
      // update in model
      means_[mah_nn_idx] = model_mean_upd;
      covariances_[mah_nn_idx] = model_cov_upd;

      PointFeature updated_point;
      updated_point.x = model_mean_upd.at<double>(0,0);
      updated_point.y = model_mean_upd.at<double>(1,0);
      updated_point.z = model_mean_upd.at<double>(2,0);

      model_ptr_->points[mah_nn_idx] = updated_point;
    }
    else
    {
      addToModel(feature_mean_tf, feature_cov_tf);
    }
  }
}

void MotionEstimationICPProbModel::publishCovariances()
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

    cv::Mat& cov = covariances_[i];
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

bool MotionEstimationICPProbModel::saveSrvCallback(
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

bool MotionEstimationICPProbModel::loadSrvCallback(
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

bool MotionEstimationICPProbModel::saveModel(const std::string& filename)
{
  // save as OpenCV yml matrix
  std::string filename_yml = filename + ".yml";

  cv::FileStorage fs(filename_yml, cv::FileStorage::WRITE);
  fs << "means"       << means_;
  fs << "covariances" << covariances_; 
  fs << "model_idx"   << model_idx_;  
  fs << "model_size"  << model_size_;       

  // save as pcd
  std::string filename_pcd = filename + ".pcd";
  pcl::PCDWriter writer;
  int result_pcd = writer.writeBinary<PointFeature>(filename_pcd, *model_ptr_);

  return (result_pcd == 0); // TODO: also OpenCV result
}

bool MotionEstimationICPProbModel::loadModel(const std::string& filename)
{
  // load OpenCV yml matrix
  std::string filename_yml = filename + ".yml";

  cv::FileStorage fs(filename_yml, cv::FileStorage::READ);
  fs["means"] >> means_;
  fs["covariances"] >> covariances_;
  fs["model_idx"] >> model_idx_;
  fs["model_size"] >> model_size_;

  // load pcd
  std::string filename_pcd = filename + ".pcd";
  pcl::PCDReader reader;
  int result_pcd = reader.read<PointFeature>(filename_pcd, *model_ptr_);
  model_ptr_->header.frame_id = fixed_frame_;

  // update the model tree
  tree_model_->setInputCloud(model_ptr_);
  reg_.setModelCloud(model_ptr_);
  reg_.setModelTree(tree_model_);

  return (result_pcd == 0); // TODO: also OpenCV result
}

} // namespace ccny_rgbd
