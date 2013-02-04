/**
 *  @file rgbd_image_proc.cpp
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
    "odom", queue_size_);
  cloud_publisher_ = nh_.advertise<PointCloudFeature>(
    "feature/cloud", 1);
  
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
  ROS_INFO("Destroying RGBD Visual Odometry"); 
}

void VisualOdometry::initParams()
{
  if (!nh_private_.getParam ("publish_tf", publish_tf_))
    publish_tf_ = true;  
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";
  if (!nh_private_.getParam ("queue_size", queue_size_))
    queue_size_ = 5;

  // detector params
  
  if (!nh_private_.getParam ("feature/publish_cloud", publish_cloud_))
    publish_cloud_ = false;
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
  
  if (!nh_private_.getParam ("reg/reg_type", reg_type_))
    reg_type_ = "ICPProbModel";
  
  if      (reg_type_ == "ICP")
    motion_estimation_ = new MotionEstimationICP(nh_, nh_private_);
  else if (reg_type_ == "ICPProbModel")
    motion_estimation_ = new MotionEstimationICPProbModel(nh_, nh_private_);
  else
    ROS_FATAL("%s is not a valid registration type!", reg_type_.c_str());
}

void VisualOdometry::resetDetector()
{  
  gft_config_server_.reset();
  star_config_server_.reset();
  orb_config_server_.reset();
  surf_config_server_.reset();
  
  if (detector_type_ == "ORB")
  { 
    ROS_INFO("Creating ORB detector");
    feature_detector_.reset(new OrbDetector());
    orb_config_server_.reset(new 
      OrbDetectorConfigServer(ros::NodeHandle(nh_private_, "feature/ORB")));
    
    // dynamic reconfigure
    OrbDetectorConfigServer::CallbackType f = boost::bind(
      &VisualOdometry::orbReconfigCallback, this, _1, _2);
    orb_config_server_->setCallback(f);
  }
  else if (detector_type_ == "SURF")
  {
    ROS_INFO("Creating SURF detector");
    feature_detector_.reset(new SurfDetector());
    surf_config_server_.reset(new 
      SurfDetectorConfigServer(ros::NodeHandle(nh_private_, "feature/SURF")));
    
    // dynamic reconfigure
    SurfDetectorConfigServer::CallbackType f = boost::bind(
      &VisualOdometry::surfReconfigCallback, this, _1, _2);
    surf_config_server_->setCallback(f);
  }
  else if (detector_type_ == "GFT")
  {
    ROS_INFO("Creating GFT detector");
    feature_detector_.reset(new GftDetector());
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
    feature_detector_.reset(new StarDetector());
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
    feature_detector_.reset(new GftDetector());
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

    motion_estimation_->setBaseToCameraTf(b2c_);
  }

  // **** create frame *************************************************

  ros::WallTime start_frame = ros::WallTime::now();
  RGBDFrame frame(rgb_msg, depth_msg, info_msg);
  ros::WallTime end_frame = ros::WallTime::now();

  // **** find features ************************************************

  ros::WallTime start_features = ros::WallTime::now();
  feature_detector_->findFeatures(frame);
  ros::WallTime end_features = ros::WallTime::now();

  // **** registration *************************************************
  
  ros::WallTime start_reg = ros::WallTime::now();
  tf::Transform motion = motion_estimation_->getMotionEstimation(frame);
  f2b_ = motion * f2b_;
  ros::WallTime end_reg = ros::WallTime::now();

  // **** publish motion **********************************************
  
  if (publish_tf_) publishTf(rgb_msg->header);
  publishOdom(rgb_msg->header);

  if (publish_cloud_) publishFeatureCloud(frame);
  
  // **** print diagnostics *******************************************

  ros::WallTime end = ros::WallTime::now();

  int n_features = frame.keypoints.size();
  int n_valid_features = frame.n_valid_keypoints;

  double d_frame    = 1000.0 * (end_frame    - start_frame   ).toSec();
  double d_features = 1000.0 * (end_features - start_features).toSec();
  double d_reg      = 1000.0 * (end_reg      - start_reg     ).toSec();
  double d_total    = 1000.0 * (end          - start         ).toSec();

  printf("[VO %d] Fr: %2.1f %s[%d][%d]: %3.1f %s %4.1f TOTAL %4.1f\n",
    frame_count_,
    d_frame, 
    detector_type_.c_str(), n_features, n_valid_features, d_features, 
    reg_type_.c_str(), d_reg, 
    d_total);

/*
  // **** for time and model size profiling
  
  float time = (rgb_msg->header.stamp - init_time_).toSec();
  int model_size = motion_estimation_->getModelSize();
  
  printf("%d\t%.2f\t%2.1f\t%2.1f\t%2.1f\t%d\t%d\n",
    frame_count_, time, 
    d_frame, d_features, d_reg, 
    n_features, model_size);
*/

/*
  // **** for position profiling

  double pos_x = f2b_.getOrigin().getX();
  double pos_y = f2b_.getOrigin().getY();
  double pos_z = f2b_.getOrigin().getZ();

  printf("%d \t %.2f \t %.3f \t %.3f \t %.3f \n",
    frame_count_, time, 
    pos_x, pos_y, pos_z);
*/

  frame_count_++;
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

void VisualOdometry::publishFeatureCloud(RGBDFrame& frame)
{
  PointCloudFeature cloud;
  cloud.header = frame.header;
  frame.constructFeaturePointCloud(cloud);   
  cloud_publisher_.publish(cloud);
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
  catch (tf::TransformException ex)
  {
    ROS_WARN("Base to camera transform unavailable %s", ex.what());
    return false;
  }

  b2c_ = tf_m;

  return true;
}

void VisualOdometry::gftReconfigCallback(GftDetectorConfig& config, uint32_t level)
{
  GftDetectorPtr gft_detector = 
    boost::static_pointer_cast<GftDetector>(feature_detector_);
    
  gft_detector->setNFeatures(config.n_features);
  gft_detector->setMinDistance(config.min_distance); 
}

void VisualOdometry::starReconfigCallback(StarDetectorConfig& config, uint32_t level)
{
  StarDetectorPtr star_detector = 
    boost::static_pointer_cast<StarDetector>(feature_detector_);
    
  star_detector->setThreshold(config.threshold);
  star_detector->setMinDistance(config.min_distance); 
}

void VisualOdometry::surfReconfigCallback(SurfDetectorConfig& config, uint32_t level)
{
  SurfDetectorPtr surf_detector = 
    boost::static_pointer_cast<SurfDetector>(feature_detector_);
    
  surf_detector->setThreshold(config.threshold);
}
    
void VisualOdometry::orbReconfigCallback(OrbDetectorConfig& config, uint32_t level)
{
  OrbDetectorPtr orb_detector = 
    boost::static_pointer_cast<OrbDetector>(feature_detector_);
    
  orb_detector->setThreshold(config.threshold);
  orb_detector->setNFeatures(config.n_features);
}

} // namespace ccny_rgbd
