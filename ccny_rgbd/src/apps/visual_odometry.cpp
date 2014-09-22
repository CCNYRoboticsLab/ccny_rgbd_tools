/**
 *  @file visual_odometry.cpp
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

#include "ccny_rgbd/apps/visual_odometry.h"

namespace ccny_rgbd {
  
VisualOdometry::VisualOdometry(
  const ros::NodeHandle& nh, 
  const ros::NodeHandle& nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false),
  frame_count_(0)
{
  ROS_INFO("Starting RGBD Visual Odometry");

  // **** initialize ROS parameters
  
  initParams();

  // **** inititialize state variables
  
  f2b_.setIdentity();

  // **** publishers

  odom_publisher_ = nh_.advertise<OdomMsg>(
    "vo", queue_size_);
  pose_stamped_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>(
    "pose", queue_size_);
  path_pub_ = nh_.advertise<PathMsg>(
    "path", queue_size_);
    
  feature_cloud_publisher_ = nh_.advertise<PointCloudFeature>(
    "feature/cloud", 1);
  feature_cov_publisher_ = nh_.advertise<visualization_msgs::Marker>(
    "feature/covariances", 1);
    
  model_cloud_publisher_ = nh_.advertise<PointCloudFeature>(
    "model/cloud", 1);
  model_cov_publisher_ = nh_.advertise<visualization_msgs::Marker>(
    "model/covariances", 1);
  
  // **** subscribers
  
  ImageTransport rgb_it(nh_);
  ImageTransport depth_it(nh_);

  sub_rgb_.subscribe(rgb_it,     "/rgbd/rgb",   queue_size_);
  sub_depth_.subscribe(depth_it, "/rgbd/depth", queue_size_);
  sub_info_.subscribe(nh_,       "/rgbd/info",  queue_size_);
  
  // Synchronize inputs.
  sync_.reset(new RGBDSynchronizer3(
                RGBDSyncPolicy3(queue_size_), sub_rgb_, sub_depth_, sub_info_));
  
  sync_->registerCallback(boost::bind(&VisualOdometry::RGBDCallback, this, _1, _2, _3));  
}

VisualOdometry::~VisualOdometry()
{
  fclose(diagnostics_file_);
  ROS_INFO("Destroying RGBD Visual Odometry"); 
}

void VisualOdometry::initParams()
{
  if (!nh_private_.getParam ("publish_tf", publish_tf_))
    publish_tf_ = true;  
  if (!nh_private_.getParam ("publish_path", publish_path_))
    publish_path_ = true;
  if (!nh_private_.getParam ("publish_odom", publish_odom_))
    publish_odom_ = true;
  if (!nh_private_.getParam ("publish_pose", publish_pose_))
    publish_pose_ = true;
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";
  if (!nh_private_.getParam ("queue_size", queue_size_))
    queue_size_ = 5;

  // detector params
  
  if (!nh_private_.getParam ("feature/publish_feature_cloud", publish_feature_cloud_))
    publish_feature_cloud_ = false;
  if (!nh_private_.getParam ("feature/publish_feature_covariances", publish_feature_cov_))
    publish_feature_cov_ = false;
  if (!nh_private_.getParam ("feature/detector_type", detector_type_))
    detector_type_ = "GFT";
  
  resetDetector();
  
  int smooth;
  double max_range, max_stdev;
  
  if (!nh_private_.getParam ("feature/smooth", smooth))
    smooth = 0;
  if (!nh_private_.getParam ("feature/max_range", max_range))
    max_range = 5.5;
  if (!nh_private_.getParam ("feature/max_stdev", max_stdev))
    max_stdev = 0.03;
  
  feature_detector_->setSmooth(smooth);
  feature_detector_->setMaxRange(max_range);
  feature_detector_->setMaxStDev(max_stdev);
  
  // registration params
  
  configureMotionEstimation();

  // diagnostic params

  if (!nh_private_.getParam("verbose", verbose_))
    verbose_ = true;
  if (!nh_private_.getParam("save_diagnostics", save_diagnostics_))
    save_diagnostics_ = false;
  if (!nh_private_.getParam("diagnostics_file_name", diagnostics_file_name_))
    diagnostics_file_name_ = "diagnostics.csv";
  
  if(save_diagnostics_)
  {
    diagnostics_file_ = fopen(diagnostics_file_name_.c_str(), "w");

    if (diagnostics_file_ == NULL)
    {
      ROS_ERROR("Can't create diagnostic file %s\n", diagnostics_file_name_.c_str());
      return;
    }

    // print header
    fprintf(diagnostics_file_, "%s, %s, %s, %s, %s, %s, %s, %s\n",
      "Frame id",
      "Frame dur.",
      "All features", "Valid features",
      "Feat extr. dur.",
      "Model points", "Registration dur.",
      "Total dur.");
  }
}

void VisualOdometry::configureMotionEstimation()
{
  int motion_constraint;

  if (!nh_private_.getParam ("reg/motion_constraint", motion_constraint))
    motion_constraint = 0;

  motion_estimation_.setMotionConstraint(motion_constraint);

  double tf_epsilon_linear;
  double tf_epsilon_angular;
  int max_iterations;
  int min_correspondences;
  int max_model_size;
  double max_corresp_dist_eucl;
  double max_assoc_dist_mah;
  int n_nearest_neighbors;   

  if (!nh_private_.getParam ("reg/ICPProbModel/tf_epsilon_linear", tf_epsilon_linear))
    tf_epsilon_linear = 1e-4; // 1 mm
  if (!nh_private_.getParam ("reg/ICPProbModel/tf_epsilon_angular", tf_epsilon_angular))
    tf_epsilon_angular = 1.7e-3; // 1 deg
  if (!nh_private_.getParam ("reg/ICPProbModel/max_iterations", max_iterations))
    max_iterations = 10;
  if (!nh_private_.getParam ("reg/ICPProbModel/min_correspondences", min_correspondences))
    min_correspondences = 15;
  if (!nh_private_.getParam ("reg/ICPProbModel/max_model_size", max_model_size))
    max_model_size = 3000;
  if (!nh_private_.getParam ("reg/ICPProbModel/max_corresp_dist_eucl", max_corresp_dist_eucl))
    max_corresp_dist_eucl = 0.15;
  if (!nh_private_.getParam ("reg/ICPProbModel/max_assoc_dist_mah", max_assoc_dist_mah))
    max_assoc_dist_mah = 10.0;
  if (!nh_private_.getParam ("reg/ICPProbModel/n_nearest_neighbors", n_nearest_neighbors))
    n_nearest_neighbors = 4;      
    
  if (!nh_private_.getParam ("reg/ICPProbModel/publish_model_cloud", publish_model_cloud_))
    publish_model_cloud_ = false;
  if (!nh_private_.getParam ("reg/ICPProbModel/publish_model_covariances", publish_model_cov_))
    publish_model_cov_ = false; 
    
  motion_estimation_.setTfEpsilonLinear(tf_epsilon_linear);
  motion_estimation_.setTfEpsilonAngular(tf_epsilon_angular);
  motion_estimation_.setMaxIterations(max_iterations);
  motion_estimation_.setMinCorrespondences(min_correspondences);
  motion_estimation_.setMaxModelSize(max_model_size);
  motion_estimation_.setMaxCorrespondenceDistEuclidean(max_corresp_dist_eucl);
  motion_estimation_.setMaxAssociationDistMahalanobis(max_assoc_dist_mah);
  motion_estimation_.setNNearestNeighbors(n_nearest_neighbors);
}

void VisualOdometry::resetDetector()
{  
  gft_config_server_.reset();
  star_config_server_.reset();
  orb_config_server_.reset();
  
  if (detector_type_ == "ORB")
  { 
    ROS_INFO("Creating ORB detector");
    feature_detector_.reset(new rgbdtools::OrbDetector());
    orb_config_server_.reset(new 
      OrbDetectorConfigServer(ros::NodeHandle(nh_private_, "feature/ORB")));
    
    // dynamic reconfigure
    OrbDetectorConfigServer::CallbackType f = boost::bind(
      &VisualOdometry::orbReconfigCallback, this, _1, _2);
    orb_config_server_->setCallback(f);
  }
  else if (detector_type_ == "GFT")
  {
    ROS_INFO("Creating GFT detector");
    feature_detector_.reset(new rgbdtools::GftDetector());
    gft_config_server_.reset(new 
      GftDetectorConfigServer(ros::NodeHandle(nh_private_, "feature/GFT")));
    
    // dynamic reconfigure
    GftDetectorConfigServer::CallbackType f = boost::bind(
      &VisualOdometry::gftReconfigCallback, this, _1, _2);
    gft_config_server_->setCallback(f);
  }
  else if (detector_type_ == "STAR")
  {
    ROS_INFO("Creating STAR detector");
    feature_detector_.reset(new rgbdtools::StarDetector());
    star_config_server_.reset(new 
      StarDetectorConfigServer(ros::NodeHandle(nh_private_, "feature/STAR")));
    
    // dynamic reconfigure
    StarDetectorConfigServer::CallbackType f = boost::bind(
      &VisualOdometry::starReconfigCallback, this, _1, _2);
    star_config_server_->setCallback(f);
  }
  else
  {
    ROS_FATAL("%s is not a valid detector type! Using GFT", detector_type_.c_str());
    feature_detector_.reset(new rgbdtools::GftDetector());
    gft_config_server_.reset(new 
      GftDetectorConfigServer(ros::NodeHandle(nh_private_, "feature/GFT")));
    
    // dynamic reconfigure
    GftDetectorConfigServer::CallbackType f = boost::bind(
      &VisualOdometry::gftReconfigCallback, this, _1, _2);
    gft_config_server_->setCallback(f);
  }
}

void VisualOdometry::RGBDCallback(
  const ImageMsg::ConstPtr& rgb_msg,
  const ImageMsg::ConstPtr& depth_msg,
  const CameraInfoMsg::ConstPtr& info_msg)
{
  ros::WallTime start = ros::WallTime::now();

  // **** initialize ***************************************************

  if (!initialized_)
  {
    initialized_ = getBaseToCameraTf(rgb_msg->header);
    init_time_ = rgb_msg->header.stamp;
    if (!initialized_) return;

    motion_estimation_.setBaseToCameraTf(eigenAffineFromTf(b2c_));
  }

  // **** create frame *************************************************

  ros::WallTime start_frame = ros::WallTime::now();
  rgbdtools::RGBDFrame frame;
  createRGBDFrameFromROSMessages(rgb_msg, depth_msg, info_msg, frame); 
  ros::WallTime end_frame = ros::WallTime::now();

  // **** find features ************************************************

  ros::WallTime start_features = ros::WallTime::now();
  feature_detector_->findFeatures(frame);
  ros::WallTime end_features = ros::WallTime::now();

  // **** registration *************************************************
  
  ros::WallTime start_reg = ros::WallTime::now();
  AffineTransform m = motion_estimation_.getMotionEstimation(frame);
  tf::Transform motion = tfFromEigenAffine(m);
  f2b_ = motion * f2b_;
  ros::WallTime end_reg = ros::WallTime::now();

  // **** publish outputs **********************************************
  
  if (publish_tf_)    publishTf(rgb_msg->header);
  if (publish_odom_)  publishOdom(rgb_msg->header);
  if (publish_path_)  publishPath(rgb_msg->header);
  if (publish_pose_)  publishPoseStamped(rgb_msg->header);
  
  if (publish_feature_cloud_) publishFeatureCloud(frame);
  if (publish_feature_cov_) publishFeatureCovariances(frame);
  
  if (publish_model_cloud_) publishModelCloud();
  if (publish_model_cov_)   publishModelCovariances();

  // **** print diagnostics *******************************************

  ros::WallTime end = ros::WallTime::now();

  frame_count_++;
  
  int n_features = frame.keypoints.size();
  int n_valid_features = frame.n_valid_keypoints;
  int n_model_pts = motion_estimation_.getModelSize();

  double d_frame    = 1000.0 * (end_frame    - start_frame   ).toSec();
  double d_features = 1000.0 * (end_features - start_features).toSec();
  double d_reg      = 1000.0 * (end_reg      - start_reg     ).toSec();
  double d_total    = 1000.0 * (end          - start         ).toSec();

  diagnostics(n_features, n_valid_features, n_model_pts,
              d_frame, d_features, d_reg, d_total);
}

void VisualOdometry::publishTf(const std_msgs::Header& header)
{
  tf::StampedTransform transform_msg(
   f2b_, header.stamp, fixed_frame_, base_frame_);
  tf_broadcaster_.sendTransform (transform_msg);
}

void VisualOdometry::publishOdom(const std_msgs::Header& header)
{
  OdomMsg odom;
  odom.header.stamp = header.stamp;
  odom.header.frame_id = fixed_frame_;
  tf::poseTFToMsg(f2b_, odom.pose.pose);
  odom_publisher_.publish(odom);
}

void VisualOdometry::publishPoseStamped(const std_msgs::Header& header)
{
  geometry_msgs::PoseStamped::Ptr pose_stamped_msg;
  pose_stamped_msg = boost::make_shared<geometry_msgs::PoseStamped>();
  pose_stamped_msg->header.stamp    = header.stamp;
  pose_stamped_msg->header.frame_id = fixed_frame_;      
  tf::poseTFToMsg(f2b_, pose_stamped_msg->pose);
  pose_stamped_publisher_.publish(pose_stamped_msg);
}


void VisualOdometry::publishPath(const std_msgs::Header& header)
{
  path_msg_.header.stamp = header.stamp;
  path_msg_.header.frame_id = fixed_frame_;

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = header.stamp;
  pose_stamped.header.frame_id = fixed_frame_;
  tf::poseTFToMsg(f2b_, pose_stamped.pose);

  path_msg_.poses.push_back(pose_stamped);
  path_pub_.publish(path_msg_);
}

bool VisualOdometry::getBaseToCameraTf(const std_msgs::Header& header)
{
  tf::StampedTransform tf_m;

  try
  {
    tf_listener_.waitForTransform(
      base_frame_, header.frame_id, header.stamp, ros::Duration(1.0));
    tf_listener_.lookupTransform (
      base_frame_, header.frame_id, header.stamp, tf_m);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN("Base to camera transform unavailable %s", ex.what());
    return false;
  }

  b2c_ = tf_m;

  return true;
}

void VisualOdometry::gftReconfigCallback(GftDetectorConfig& config, uint32_t level)
{
  rgbdtools::GftDetectorPtr gft_detector = 
    boost::static_pointer_cast<rgbdtools::GftDetector>(feature_detector_);
    
  gft_detector->setNFeatures(config.n_features);
  gft_detector->setMinDistance(config.min_distance); 
}

void VisualOdometry::starReconfigCallback(StarDetectorConfig& config, uint32_t level)
{
  rgbdtools::StarDetectorPtr star_detector = 
    boost::static_pointer_cast<rgbdtools::StarDetector>(feature_detector_);
    
  star_detector->setThreshold(config.threshold);
  star_detector->setMinDistance(config.min_distance); 
}
    
void VisualOdometry::orbReconfigCallback(OrbDetectorConfig& config, uint32_t level)
{
  rgbdtools::OrbDetectorPtr orb_detector = 
    boost::static_pointer_cast<rgbdtools::OrbDetector>(feature_detector_);
    
  orb_detector->setThreshold(config.threshold);
  orb_detector->setNFeatures(config.n_features);
}

void VisualOdometry::diagnostics(
  int n_features, int n_valid_features, int n_model_pts,
  double d_frame, double d_features, double d_reg, double d_total)
{
  if(save_diagnostics_ && diagnostics_file_ != NULL)
  {
    // print to file
    fprintf(diagnostics_file_, "%d, %2.1f, %d, %3.1f, %d, %4.1f, %4.1f\n",
      frame_count_,
      d_frame,
      n_valid_features, d_features,
      n_model_pts, d_reg,
      d_total);
  }
  if (verbose_)
  {
    // print to screen
    ROS_INFO("[VO %d] %s[%d]: %.1f Reg[%d]: %.1f TOT: %.1f\n",
      frame_count_,
      detector_type_.c_str(), n_valid_features, d_features,
      n_model_pts, d_reg,
      d_total);
  }

  return;
}

void VisualOdometry::publishFeatureCloud(rgbdtools::RGBDFrame& frame)
{
  PointCloudFeature feature_cloud; 
  frame.constructFeaturePointCloud(feature_cloud);   
  feature_cloud_publisher_.publish(feature_cloud);
}

void VisualOdometry::publishFeatureCovariances(rgbdtools::RGBDFrame& frame)
{
  ///< @TODO publish feature covariances
}

void VisualOdometry::publishModelCloud()
{
  PointCloudFeature::Ptr model_cloud_ptr = motion_estimation_.getModel();
  model_cloud_ptr->header.frame_id = fixed_frame_;
  model_cloud_publisher_.publish(model_cloud_ptr);
}

void VisualOdometry::publishModelCovariances()
{
  ///< @TODO ppublish model covariances
}

} // namespace ccny_rgbd
