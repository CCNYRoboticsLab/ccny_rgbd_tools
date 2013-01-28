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
    "odom", 5);

  // **** subscribers
  
  image_transport::ImageTransport rgb_it(nh_);
  image_transport::ImageTransport depth_it(nh_);

  sub_depth_.subscribe(depth_it, "/rgbd/depth", queue_size_);
  sub_rgb_.subscribe(rgb_it, "/rgbd/rgb", queue_size_);
  sub_info_.subscribe(nh_, "/rgbd/info", queue_size_);

  // Synchronize inputs.
  sync_.reset(new RGBDSynchronizer3(
                RGBDSyncPolicy3(queue_size_), sub_rgb_, sub_depth_, sub_info_));
  
  sync_->registerCallback(boost::bind(&VisualOdometry::imageCb, this, _1, _2, _3));  
}

VisualOdometry::~VisualOdometry()
{
  ROS_INFO("Destroying RGBD Visual Odometry"); 

  delete feature_detector_;
}

void VisualOdometry::initParams()
{
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";
  if (!nh_private_.getParam ("queue_size", queue_size_))
    queue_size_ = 5;

  if (!nh_private_.getParam ("feature/detector_type", detector_type_))
    detector_type_ = "GFT";
  if (!nh_private_.getParam ("reg/reg_type", reg_type_))
    reg_type_ = "ICP";
  
  // feature params
  if      (detector_type_ == "ORB")
    feature_detector_ = new OrbDetector(nh_, nh_private_);
  else if (detector_type_ == "SURF")
    feature_detector_ = new SurfDetector(nh_, nh_private_);
  else if (detector_type_ == "GFT")
    feature_detector_ = new GftDetector(nh_, nh_private_);
  else if (detector_type_ == "STAR")
    feature_detector_ = new StarDetector(nh_, nh_private_);
  else
    ROS_FATAL("%s is not a valid detector type!", detector_type_.c_str());

  // registration params
  if      (reg_type_ == "ICP")
    motion_estimation_ = new MotionEstimationICP(nh_, nh_private_);
  else if (reg_type_ == "ICPProbModel")
    motion_estimation_ = new MotionEstimationICPProbModel(nh_, nh_private_);
  else
    ROS_FATAL("%s is not a valid registration type!", reg_type_.c_str());
}

void VisualOdometry::imageCb(
  const sensor_msgs::ImageConstPtr& depth_msg,
  const sensor_msgs::ImageConstPtr& rgb_msg,
  const sensor_msgs::CameraInfoConstPtr& info_msg)
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

  publishTf(rgb_msg->header);

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

  OdomMsg odom;
  odom.header.stamp = header.stamp;
  odom.header.frame_id = fixed_frame_;
  tf::poseTFToMsg(f2b_, odom.pose.pose);
  odom_publisher_.publish(odom);
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

} //namespace ccny_rgbd
