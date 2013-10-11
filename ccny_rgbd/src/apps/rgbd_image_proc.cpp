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

#include "ccny_rgbd/apps/rgbd_image_proc.h"

namespace ccny_rgbd {

RGBDImageProc::RGBDImageProc(
  const ros::NodeHandle& nh, 
  const ros::NodeHandle& nh_private):
  nh_(nh), nh_private_(nh_private), 
  rgb_image_transport_(nh_),
  depth_image_transport_(nh_), 
  config_server_(nh_private_),
  size_in_(0,0)
{ 
  // parameters 
  if (!nh_private_.getParam ("queue_size", queue_size_))
    queue_size_ = 5;
  if (!nh_private_.getParam("scale", scale_))
    scale_ = 1.0;
  if (!nh_private_.getParam("unwarp", unwarp_))
    unwarp_ = true;
  if (!nh_private_.getParam("verbose", verbose_))
    verbose_ = false;
  if (!nh_private_.getParam("publish_cloud", publish_cloud_))
    publish_cloud_ = true;
  if (!nh_private_.getParam("calib_path", calib_path_))
  {
    std::string home_path = getenv("HOME");
    calib_path_ = home_path + "./ros/rgbd_calibration";
  }

  calib_extr_filename_ = calib_path_ + "/extr.yml";
  calib_warp_filename_ = calib_path_ + "/warp.yml";
  
  // load calibration (extrinsics, depth unwarp params) from files
  loadCalibration();
  if (unwarp_)
  {
    bool load_result = loadUnwarpCalibration();
    if (!load_result)
    {
      ROS_WARN("Disbaling unwarping due to missing calibration file");
      unwarp_ = true;
    }
  }
  
  // publishers
  rgb_publisher_   = rgb_image_transport_.advertise(
    "rgbd/rgb", queue_size_);
  depth_publisher_ = depth_image_transport_.advertise(
    "rgbd/depth", queue_size_);
  info_publisher_  = nh_.advertise<CameraInfoMsg>(
    "rgbd/info", queue_size_);

  if(publish_cloud_)
    cloud_publisher_ = nh_.advertise<PointCloudT>(
          "rgbd/cloud", queue_size_);

  // dynamic reconfigure
  ProcConfigServer::CallbackType f = boost::bind(&RGBDImageProc::reconfigCallback, this, _1, _2);
  config_server_.setCallback(f);
  
  // subscribers
  sub_rgb_.subscribe  (rgb_image_transport_,   
    "/camera/rgb/image_color", queue_size_);
  sub_depth_.subscribe(depth_image_transport_, 
    "/camera/depth/image_raw", queue_size_); //16UC1
  
  sub_rgb_info_.subscribe  (nh_, 
    "/camera/rgb/camera_info", queue_size_);
  sub_depth_info_.subscribe(nh_, 
    "/camera/depth/camera_info", queue_size_);
  
  sync_.reset(new RGBDSynchronizer4(
                RGBDSyncPolicy4(queue_size_), sub_rgb_, sub_depth_, 
                sub_rgb_info_, sub_depth_info_));
  
  sync_->registerCallback(boost::bind(&RGBDImageProc::RGBDCallback, this, _1, _2, _3, _4)); 
}

RGBDImageProc::~RGBDImageProc()
{
  ROS_INFO("Destroying RGBDImageProc"); 
}

bool RGBDImageProc::loadCalibration()
{
  // check that file exist
  if (!boost::filesystem::exists(calib_extr_filename_))
  {
    ROS_ERROR("Could not open %s", calib_extr_filename_.c_str());
    /// @todo handle this with default ir2rgb
    return false;
  }

  // load intrinsics and distortion coefficients
  ROS_INFO("Reading camera calibration files...");
  cv::FileStorage fs_extr(calib_extr_filename_, cv::FileStorage::READ);

  fs_extr["ir2rgb"] >> ir2rgb_;

  return true;
}

bool RGBDImageProc::loadUnwarpCalibration()
{
  if (!boost::filesystem::exists(calib_warp_filename_))
  {
    ROS_ERROR("Could not open %s", calib_warp_filename_.c_str());
    return false;
  }
  
  // load warp coefficients
  cv::FileStorage fs_warp(calib_warp_filename_, cv::FileStorage::READ);   

  fs_warp["c0"] >> coeff_0_;
  fs_warp["c1"] >> coeff_1_;
  fs_warp["c2"] >> coeff_2_;
  fs_warp["fit_mode"] >> fit_mode_;
  
  return true;
}

void RGBDImageProc::initMaps(
  const CameraInfoMsg::ConstPtr& rgb_info_msg,
  const CameraInfoMsg::ConstPtr& depth_info_msg)
{ 
  boost::mutex::scoped_lock(mutex_);
  
  ROS_INFO("Initializing rectification maps");
  
  // **** get OpenCV matrices from CameraInfo messages
  cv::Mat intr_rgb, intr_depth;
  cv::Mat dist_rgb, dist_depth;
  
  convertCameraInfoToMats(rgb_info_msg, intr_rgb, dist_rgb);
  convertCameraInfoToMats(depth_info_msg, intr_depth, dist_depth); 
  
  // **** sizes 
  double alpha = 0.0;
    
  if (size_in_.width  != (int)rgb_info_msg->width || 
      size_in_.height != (int)rgb_info_msg->height)
  {
    ROS_WARN("Image size does not match CameraInfo size. Rescaling.");
    double w_factor = (double)size_in_.width  / (double)rgb_info_msg->width;
    double h_factor = (double)size_in_.height / (double)rgb_info_msg->height;
    
    intr_rgb.at<double>(0,0) *= w_factor;
    intr_rgb.at<double>(1,1) *= h_factor;   
    intr_rgb.at<double>(0,2) *= w_factor;
    intr_rgb.at<double>(1,2) *= h_factor;

    intr_depth.at<double>(0,0) *= w_factor;
    intr_depth.at<double>(1,1) *= h_factor;   
    intr_depth.at<double>(0,2) *= w_factor;
    intr_depth.at<double>(1,2) *= h_factor;
  }
  
  cv::Size size_out;
  size_out.height = size_in_.height * scale_;
  size_out.width  = size_in_.width  * scale_;
   
  // **** get optimal camera matrices
  intr_rect_rgb_ = cv::getOptimalNewCameraMatrix(
    intr_rgb, dist_rgb, size_in_, alpha, size_out);
 
  intr_rect_depth_ = cv::getOptimalNewCameraMatrix(
    intr_depth, dist_depth, size_in_, alpha, size_out);
      
  // **** create undistortion maps
  cv::initUndistortRectifyMap(
    intr_rgb, dist_rgb, cv::Mat(), intr_rect_rgb_, 
    size_out, CV_16SC2, map_rgb_1_, map_rgb_2_);
  
  cv::initUndistortRectifyMap(
    intr_depth, dist_depth, cv::Mat(), intr_rect_depth_, 
    size_out, CV_16SC2, map_depth_1_, map_depth_2_);  
  
  // **** rectify the coefficient images
  if(unwarp_)
  {
    cv::remap(coeff_0_, coeff_0_rect_, map_depth_1_, map_depth_2_,  cv::INTER_NEAREST);
    cv::remap(coeff_1_, coeff_1_rect_, map_depth_1_, map_depth_2_,  cv::INTER_NEAREST);
    cv::remap(coeff_2_, coeff_2_rect_, map_depth_1_, map_depth_2_,  cv::INTER_NEAREST);
  }

  // **** save new intrinsics as camera models
  rgb_rect_info_msg_.header = rgb_info_msg->header;
  rgb_rect_info_msg_.width  = size_out.width;
  rgb_rect_info_msg_.height = size_out.height;  

  depth_rect_info_msg_.header = depth_info_msg->header;
  depth_rect_info_msg_.width  = size_out.width;
  depth_rect_info_msg_.height = size_out.height;  
  
  convertMatToCameraInfo(intr_rect_rgb_,   rgb_rect_info_msg_);
  convertMatToCameraInfo(intr_rect_depth_, depth_rect_info_msg_);  
}

void RGBDImageProc::RGBDCallback(
  const ImageMsg::ConstPtr& rgb_msg,
  const ImageMsg::ConstPtr& depth_msg,
  const CameraInfoMsg::ConstPtr& rgb_info_msg,
  const CameraInfoMsg::ConstPtr& depth_info_msg)
{  
  boost::mutex::scoped_lock(mutex_);
  
  // for profiling
  double dur_unwarp, dur_rectify, dur_reproject, dur_cloud, dur_allocate; 
  
  // **** images need to be the same size
  if (rgb_msg->height != depth_msg->height || 
      rgb_msg->width  != depth_msg->width)
  {
    ROS_WARN("RGB and depth images have different sizes, skipping");
    return;
  }
  
  // **** initialize if needed
  if (size_in_.height != (int)rgb_msg->height ||
      size_in_.width  != (int)rgb_msg->width)
  {
    ROS_INFO("Initializing");
  
    size_in_.height = (int)rgb_msg->height;
    size_in_.width  = (int)rgb_msg->width;
    
    initMaps(rgb_info_msg, depth_info_msg);
  }
  
  // **** convert ros images to opencv Mat
  cv_bridge::CvImageConstPtr rgb_ptr   = cv_bridge::toCvShare(rgb_msg);
  cv_bridge::CvImageConstPtr depth_ptr = cv_bridge::toCvShare(depth_msg);

  const cv::Mat& rgb_img   = rgb_ptr->image;
  const cv::Mat& depth_img = depth_ptr->image;
  
  //cv::imshow("RGB", rgb_img);
  //cv::imshow("Depth", depth_img);
  //cv::waitKey(1);
  
  // **** rectify
  ros::WallTime start_rectify = ros::WallTime::now();
  cv::Mat rgb_img_rect, depth_img_rect;
  cv::remap(rgb_img, rgb_img_rect, map_rgb_1_, map_rgb_2_, cv::INTER_LINEAR);
  cv::remap(depth_img, depth_img_rect, map_depth_1_, map_depth_2_,  cv::INTER_NEAREST);
  dur_rectify = getMsDuration(start_rectify);
  
  //cv::imshow("RGB Rect", rgb_img_rect);
  //cv::imshow("Depth Rect", depth_img_rect);
  //cv::waitKey(1);
  
  // **** unwarp 
  if (unwarp_) 
  {    
    ros::WallTime start_unwarp = ros::WallTime::now();
    rgbdtools::unwarpDepthImage(
      depth_img_rect, coeff_0_rect_, coeff_1_rect_, coeff_2_rect_, fit_mode_);
    dur_unwarp = getMsDuration(start_unwarp);
  }
  else dur_unwarp = 0.0;
  
  // **** reproject
  ros::WallTime start_reproject = ros::WallTime::now();
  cv::Mat depth_img_rect_reg;
  rgbdtools::buildRegisteredDepthImage(
    intr_rect_depth_, intr_rect_rgb_, ir2rgb_, depth_img_rect, depth_img_rect_reg);
  dur_reproject = getMsDuration(start_reproject);

  // **** point cloud
  if (publish_cloud_)
  {
    ros::WallTime start_cloud = ros::WallTime::now();
    PointCloudT::Ptr cloud_ptr;
    cloud_ptr.reset(new PointCloudT());
    rgbdtools::buildPointCloud(
      depth_img_rect_reg, rgb_img_rect, intr_rect_rgb_, *cloud_ptr);
    // The point cloud timestamp, int usec.
    cloud_ptr->header.stamp = rgb_info_msg->header.stamp.toNSec() * 1e-3;
    cloud_ptr->header.frame_id = rgb_info_msg->header.frame_id;
    cloud_publisher_.publish(cloud_ptr);
    dur_cloud = getMsDuration(start_cloud);
  }
  else dur_cloud = 0.0;
  
  // **** allocate registered rgb image
  ros::WallTime start_allocate = ros::WallTime::now();

  cv_bridge::CvImage cv_img_rgb(rgb_msg->header, rgb_msg->encoding, rgb_img_rect);
  ImageMsg::Ptr rgb_out_msg = cv_img_rgb.toImageMsg();

  // **** allocate registered depth image
  cv_bridge::CvImage cv_img_depth(depth_msg->header, depth_msg->encoding, depth_img_rect_reg);
  ImageMsg::Ptr depth_out_msg = cv_img_depth.toImageMsg();
  
  // **** update camera info (single, since both images are in rgb frame)
  rgb_rect_info_msg_.header = rgb_info_msg->header;
  
  dur_allocate = getMsDuration(start_allocate); 

  // **** print diagnostics
  
  double dur_total = dur_rectify + dur_reproject + dur_unwarp + dur_cloud + dur_allocate;
  if(verbose_)
  {
    ROS_INFO("Rect %.1f Reproj %.1f Unwarp %.1f Cloud %.1f Alloc %.1f Total %.1f ms",
             dur_rectify, dur_reproject,  dur_unwarp, dur_cloud, dur_allocate,
             dur_total);
  }
  // **** publish
  rgb_publisher_.publish(rgb_out_msg);
  depth_publisher_.publish(depth_out_msg);
  info_publisher_.publish(rgb_rect_info_msg_);
}

void RGBDImageProc::reconfigCallback(ProcConfig& config, uint32_t level)
{
  boost::mutex::scoped_lock(mutex_);
  bool old_publish_cloud = publish_cloud_;
      publish_cloud_ = config.publish_cloud;
  if(!old_publish_cloud && publish_cloud_)
  {
    cloud_publisher_ = nh_.advertise<PointCloudT>(
        "rgbd/cloud", queue_size_);
  }
  else
  {
    if(old_publish_cloud && !publish_cloud_)
      cloud_publisher_.shutdown();
  }


  scale_ = config.scale;
  size_in_ = cv::Size(0,0); // force a reinitialization on the next image callback
  ROS_INFO("Resampling scale set to %.2f", scale_);
}

} //namespace ccny_rgbd
