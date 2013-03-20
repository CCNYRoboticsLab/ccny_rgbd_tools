#include "ccny_rgbd_calibrate/logger.h"

namespace ccny_rgbd {

Logger::Logger(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), nh_private_(nh_private)
{
  // **** parameters

  if (!nh_private_.getParam ("n_rgb", n_rgb_))
    n_rgb_ = 1000;
  if (!nh_private_.getParam ("n_depth", n_depth_))
    n_depth_ = 1000;
  if (!nh_private_.getParam ("n_ir", n_ir_))
    n_ir_ = 0;
  if (!nh_private_.getParam ("id", id_))
    id_ = 0;
  if (!nh_private_.getParam ("path", path_))
    ROS_ERROR("path param needs to be set");

  // ensures directories exist
  prepareDirectories();
  
  // **** subscribers
  
  int queue_size = 5;
  
  if (n_ir_ > 0)
  {
    ROS_INFO("Subscribing to IR");
    
    image_transport::ImageTransport ir_it(nh_);
    sub_ir_ = nh_.subscribe( "/camera/ir/image_raw", 1,  &Logger::IRCallback, this);
    cv::namedWindow("IR", 1);
  }
  else
  {
    ROS_INFO("Subscribing to RGBD");
      
    image_transport::ImageTransport rgb_it(nh_);
    image_transport::ImageTransport depth_it(nh_);
    
    sub_depth_.subscribe(depth_it, "/camera/depth/image_raw", 1);
    sub_rgb_.subscribe(rgb_it, "/camera/rgb/image_color", 1);
    sub_info_.subscribe(nh_, "/camera/rgb/camera_info", 1);
    
    // Synchronize inputs.
    sync_.reset(new RGBDSynchronizer3(
                RGBDSyncPolicy3(queue_size), sub_rgb_, sub_depth_, sub_info_));
    sync_->registerCallback(boost::bind(&Logger::RGBDCallback, this, _1, _2, _3));  
    
    cv::namedWindow("RGB", 1);
    cv::namedWindow("Depth", 1);
  }
}

Logger::~Logger()
{
  ROS_INFO("Destroying RGBD Visual Odometry"); 
}

void Logger::prepareDirectories()
{  
  boost::filesystem::create_directory(path_);
  ROS_INFO("Creating directory: %s", path_.c_str());
  
  rgb_path_   = path_ + "/rgb/";
  depth_path_ = path_ + "/depth/";
  ir_path_    = path_ + "/ir/";

  boost::filesystem::create_directory(rgb_path_); 
  boost::filesystem::create_directory(depth_path_); 
  boost::filesystem::create_directory(ir_path_); 
}

void Logger::IRCallback(
  const sensor_msgs::ImageConstPtr& ir_msg)
{
  std::stringstream ss_filename;
  ss_filename << std::setw(4) << std::setfill('0') << id_ << ".png";

  if (id_< n_ir_)
  {
    cv_bridge::CvImagePtr ir_ptr = cv_bridge::toCvCopy(ir_msg);

    cv::Mat ir;
    ir_ptr->image.convertTo(ir, CV_8UC1);
    
    cv::imwrite(ir_path_+ ss_filename.str(), ir);
        
    ROS_INFO("IR %s saved", ss_filename.str().c_str());

    cv::imshow("IR", ir);
    cv::waitKey(1);
  } 
  
  id_++;
}

void Logger::RGBDCallback(
  const sensor_msgs::ImageConstPtr& depth_msg,
  const sensor_msgs::ImageConstPtr& rgb_msg,
  const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  std::stringstream ss_filename;
  ss_filename << std::setw(4) << std::setfill('0') << id_ << ".png";

  if (id_< n_rgb_)
  {
    cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(rgb_msg);
    cv::imwrite(rgb_path_ + ss_filename.str(), rgb_ptr->image);
    ROS_INFO("RGB %s saved", ss_filename.str().c_str());

    cv::imshow("RGB", rgb_ptr->image);
    cv::waitKey(1);
  }
  if (id_< n_depth_)
  {
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg);
    cv::imwrite(depth_path_ + ss_filename.str(), depth_ptr->image);
    ROS_INFO("Depth %s saved", ss_filename.str().c_str());
    
    cv::imshow("Depth", depth_ptr->image);
    cv::waitKey(1);
  }
  id_++;
}

} //namespace ccny_rgbd
