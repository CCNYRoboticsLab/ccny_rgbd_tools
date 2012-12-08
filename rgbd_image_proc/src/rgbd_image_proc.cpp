#include "rgbd_image_proc/rgbd_image_proc.h"

namespace ccny_rgbd {

RGBDImageProc::RGBDImageProc(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), nh_private_(nh_private), 
  rgb_image_transport_(nh_),
  depth_image_transport_(nh_), 
  initialized_(false)
{
  // parameters 
  if (!nh_private_.getParam ("scale", scale_))
    scale_ = 1.0;
  
  std::string home_path = getenv("HOME");
  calib_path_ = home_path + "/ros/ccny-ros-pkg/ccny_rgbd_data/images/ext_calib_01/";

  calib_extr_filename_ = calib_path_ + "extr.yml";
  calib_warp_filename_ = calib_path_ + "warp.yml";
  
  // load calibration (extrinsics, depth unwarp params) from files
  loadCalibration();
  
  // publishers
  rgb_publisher_   = rgb_image_transport_.advertise("rgbd/rgb", 1);
  depth_publisher_ = depth_image_transport_.advertise("rgbd/depth", 1);
  info_publisher_  = nh_.advertise<CameraInfoMsg>("rgbd/info", 1);
  cloud_publisher_ = nh_.advertise<PointCloudT>("rgbd/cloud", 1);

  // dynamic reconfigure
  ProcConfigServer::CallbackType f = boost::bind(&RGBDImageProc::reconfigCallback, this, _1, _2);
  config_server_.setCallback(f);
  
  // subscribers
  int queue_size = 5;
     
  // TODO: queue size in subscribers?
  sub_rgb_.subscribe  (rgb_image_transport_,   "/camera/rgb/image_color", 1);
  sub_depth_.subscribe(depth_image_transport_, "/camera/depth/image_raw", 1); //16UC1
  
  sub_rgb_info_.subscribe  (nh_, "/camera/rgb/camera_info",   1);
  sub_depth_info_.subscribe(nh_, "/camera/depth/camera_info", 1);
  
  sync_.reset(new RGBDSynchronizer(
                RGBDSyncPolicy(queue_size), sub_rgb_, sub_depth_, 
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
    // TODO: handle this with default ir2rgb
    return false;
  }
  if (!boost::filesystem::exists(calib_warp_filename_))
  {
    ROS_ERROR("Could not open %s", calib_warp_filename_.c_str());
    // TODO: handle this with default warp coeff
    return false;
  }

  // **** load intrinsics and distortion coefficients
  ROS_INFO("Reading camera calibration files...");
  cv::FileStorage fs_extr(calib_extr_filename_, cv::FileStorage::READ);
  cv::FileStorage fs_warp(calib_warp_filename_, cv::FileStorage::READ);   
  
  fs_extr["ir2rgb"] >> ir2rgb_;
  fs_warp["c0"] >> coeff_0_;
  fs_warp["c1"] >> coeff_1_;
  fs_warp["c2"] >> coeff_2_;
  fs_warp["fit_mode"] >> fit_mode_;

  return true;
}

void RGBDImageProc::convertCameraInfoToMats(
  const CameraInfoMsg::ConstPtr camera_info_msg,
  cv::Mat& intr,
  cv::Mat& dist)
{
  // set intrinsic matrix from K vector
  intr = cv::Mat(3, 3, CV_64FC1);
  for (int idx = 0; idx < 9; ++idx)
  {
    int i = idx % 3;
    int j = idx / 3;
    intr.at<double>(j, i) = camera_info_msg->K[idx];
  }
  
  // set distortion matrix from D vector
  int d_size = camera_info_msg->D.size();
  dist = cv::Mat(1, d_size, CV_64FC1);
  for (int idx = 0; idx < d_size; ++idx)
  {
    dist.at<double>(0, idx) = camera_info_msg->D[idx];   
  }
}

/* sets the info message to a given intrinsic matrix,
 * with D = 0 and R = identity
 */
void RGBDImageProc::convertMatToCameraInfo(
  const cv::Mat& intr,
  CameraInfoMsg& camera_info_msg)
{
  // set D matrix to 0
  camera_info_msg.D.resize(5);
  std::fill(camera_info_msg.D.begin(), camera_info_msg.D.end(), 0.0);
  
  // set K matrix to optimal new camera matrix
  for (int i = 0; i < 3; ++i)
  for (int j = 0; j < 3; ++j)
    camera_info_msg.K[j*3 + i] = intr.at<double>(j,i);
  
  // set R matrix to identity
  std::fill(camera_info_msg.R.begin(), camera_info_msg.R.end(), 0.0);  
  camera_info_msg.R[0*3 + 0] = 1.0;
  camera_info_msg.R[1*3 + 1] = 1.0;
  camera_info_msg.R[2*3 + 2] = 1.0;
    
  //set P matrix to K
  std::fill(camera_info_msg.P.begin(), camera_info_msg.P.end(), 0.0);  
  for (int i = 0; i < 3; ++i)
  for (int j = 0; j < 3; ++j)
    camera_info_msg.P[j*4 + i] = intr.at<double>(j,i);
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
    
  size_in_.width  = rgb_info_msg->width;
  size_in_.height = rgb_info_msg->height;
  
  size_out_.width  = size_in_.width  * scale_;
  size_out_.height = size_in_.height * scale_;
   
  // **** get optimal camera matrices
  intr_rect_rgb_ = cv::getOptimalNewCameraMatrix(
    intr_rgb, dist_rgb, size_in_, alpha, size_out_);
 
  intr_rect_depth_ = cv::getOptimalNewCameraMatrix(
    intr_depth, dist_depth, size_in_, alpha, size_out_);
      
  // **** create undistortion maps
  cv::initUndistortRectifyMap(
    intr_rgb, dist_rgb, cv::Mat(), intr_rect_rgb_, 
    size_out_, CV_16SC2, map_rgb_1_, map_rgb_2_);
  
  cv::initUndistortRectifyMap(
    intr_depth, dist_depth, cv::Mat(), intr_rect_depth_, 
    size_out_, CV_16SC2, map_depth_1_, map_depth_2_);  
  
  // **** rectify the coefficient images
  cv::remap(coeff_0_, coeff_0_rect_, map_depth_1_, map_depth_2_,  cv::INTER_NEAREST);
  cv::remap(coeff_1_, coeff_1_rect_, map_depth_1_, map_depth_2_,  cv::INTER_NEAREST);
  cv::remap(coeff_2_, coeff_2_rect_, map_depth_1_, map_depth_2_,  cv::INTER_NEAREST);

  // **** save new intrinsics as camera models
  rgb_rect_info_msg_.header = rgb_info_msg->header;
  rgb_rect_info_msg_.width  = size_out_.width;
  rgb_rect_info_msg_.height = size_out_.height;  

  depth_rect_info_msg_.header = depth_info_msg->header;
  depth_rect_info_msg_.width  = size_out_.width;
  depth_rect_info_msg_.height = size_out_.height;  
  
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
  
  // **** initialize if needed
  if (!initialized_)
  {
    initMaps(rgb_info_msg, depth_info_msg);
    initialized_ = true;
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
  double dur_rectify = getMsDuration(start_rectify);
  
  //cv::imshow("RGB Rect", rgb_img_rect);
  //cv::imshow("Depth Rect", depth_img_rect);
  //cv::waitKey(1);
  
  // **** unwarp 
  ros::WallTime start_unwarp = ros::WallTime::now();
  unwarpDepthImage(depth_img_rect, coeff_0_rect_, coeff_1_rect_, coeff_2_rect_, fit_mode_);
  double dur_unwarp = getMsDuration(start_unwarp);
   
  // **** reproject
  ros::WallTime start_reproject = ros::WallTime::now();
  cv::Mat depth_img_rect_reg;
  buildRegisteredDepthImage(intr_rect_depth_, intr_rect_rgb_, ir2rgb_,
                            depth_img_rect, depth_img_rect_reg);
  double dur_reproject = getMsDuration(start_reproject);


  // **** point cloud
  ros::WallTime start_cloud = ros::WallTime::now();
  PointCloudT::Ptr cloud;
  cloud.reset(new PointCloudT());
  buildPointCloud(depth_img_rect_reg, rgb_img_rect, intr_rect_rgb_, *cloud);
  cloud->header = rgb_info_msg->header;
  cloud_publisher_.publish(cloud);
  double dur_cloud = getMsDuration(start_cloud);
  
  // **** allocate registered rgb image
  ros::WallTime start_allocate = ros::WallTime::now();

  cv_bridge::CvImage cv_img_rgb(rgb_msg->header, rgb_msg->encoding, rgb_img_rect);
  ImageMsg::Ptr rgb_out_msg = cv_img_rgb.toImageMsg();

  // **** allocate registered depth image
  cv_bridge::CvImage cv_img_depth(depth_msg->header, depth_msg->encoding, depth_img_rect_reg);
  ImageMsg::Ptr depth_out_msg = cv_img_depth.toImageMsg();
  
  // **** update camera info (single, since both images are in rgb frame)
  rgb_rect_info_msg_.header = rgb_info_msg->header;
  
  double dur_allocate = getMsDuration(start_allocate); 

  //ROS_INFO("Rect: %.1f Reproj: %.1f Unwarp: %.1f Cloud %.1f Alloc: %.1f ms", 
  //  dur_rectify, dur_reproject,  dur_unwarp, dur_cloud, dur_allocate);

  // **** publish
  rgb_publisher_.publish(rgb_out_msg);
  depth_publisher_.publish(depth_out_msg);
  info_publisher_.publish(rgb_rect_info_msg_);
}

void RGBDImageProc::reconfigCallback(ProcConfig& config, uint32_t level)
{
  boost::mutex::scoped_lock(mutex_);
  initialized_ = false;
  scale_ = config.scale;
  ROS_INFO("Resampling scale set to %.2f", scale_);
}

} //namespace ccny_rgbd
