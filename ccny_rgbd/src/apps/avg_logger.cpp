#include "ccny_rgbd/apps/avg_logger.h"

namespace ccny_rgbd {

AvgLogger::AvgLogger(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), nh_private_(nh_private), count_(0)
{
  // **** parameters
  
  if (!nh_private_.getParam ("n_depth", n_depth_))
    n_depth_ = 1000;
  if (!nh_private_.getParam ("id", id_))
    id_ = 0;
  if (!nh_private_.getParam ("sequence", sequence_))
    sequence_ = "rgbd";

  ss_filename_ << std::setw(4) << std::setfill('0') << id_ << ".png";
  
  ROS_INFO("Depth images: %d, id is: %d", n_depth_, id_);
  
  // ensures directories exist
  prepareDirectories();
  
  // **** subscribers
  
  int queue_size = 5;
  
  ROS_INFO("Subscribing to RGBD");
    
  image_transport::ImageTransport rgb_it(nh_);
  image_transport::ImageTransport depth_it(nh_);
  
  sub_depth_.subscribe(depth_it, "/camera/depth/image", 1);
  sub_rgb_.subscribe(rgb_it, "/camera/rgb/image_color", 1);
  sub_info_.subscribe(nh_, "/camera/rgb/camera_info", 1);
  
  sync_.reset(new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_));
  sync_->registerCallback(boost::bind(&AvgLogger::RGBDCallback, this, _1, _2, _3));  
   
  depth_cnt_img_ = cv::Mat::zeros(480, 640, CV_16UC1);
  depth_sum_img_ = cv::Mat::zeros(480, 640, CV_64FC1);
}

AvgLogger::~AvgLogger()
{
  ROS_INFO("Destroying RGBD Visual Odometry"); 
}

void AvgLogger::prepareDirectories()
{
  std::stringstream ss_seq_path;
   
  ss_seq_path << getenv("HOME") << "/ros/images/" << sequence_;
  
  ROS_INFO("Creating directory: %s", ss_seq_path.str().c_str());
  boost::filesystem::create_directory(ss_seq_path.str());
  
  ss_rgb_path_   << ss_seq_path.str() << "/rgb/";
  ss_depth_path_ << ss_seq_path.str() << "/depth/";

  boost::filesystem::create_directory(ss_rgb_path_.str()); 
  boost::filesystem::create_directory(ss_depth_path_.str()); 
}

void AvgLogger::RGBDCallback(
  const sensor_msgs::ImageConstPtr& depth_msg,
  const sensor_msgs::ImageConstPtr& rgb_msg,
  const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if (count_ > n_depth_) return;
  ROS_INFO("RGBD Callback %d", count_);
  
  // save 1 rgb image 
  if (count_ == 0)
  {
    std::string rgb_filename = ss_rgb_path_.str() + ss_filename_.str();
    
    cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(rgb_msg);
    cv::imwrite(rgb_filename, rgb_ptr->image);
    ROS_INFO("RGB %s saved", rgb_filename.c_str());
  }
  
  // accumulate depth images
  if (count_< n_depth_)
  {
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg);

    cv::Mat& depth_img = depth_ptr->image;
    
    for (int u = 0; u < depth_img.cols; ++u)
    for (int v = 0; v < depth_img.rows; ++v)
    {
      float z = depth_img.at<float>(v, u);
      
      if (!isnan(z))
      {
        depth_sum_img_.at<double>(v, u)   += z;
        depth_cnt_img_.at<uint16_t>(v, u) += 1;
      }
    }
  }
  else if (count_ == n_depth_)
  {
    cv::Mat depth_avg_img = cv::Mat::zeros(480, 640, CV_16UC1);
    
    for (int u = 0; u < depth_avg_img.cols; ++u)
    for (int v = 0; v < depth_avg_img.rows; ++v)
    {
      double z = depth_sum_img_.at<double>(v, u);
      int    c = depth_cnt_img_.at<uint16_t>(v, u);
      
      if (c > 0)
      {
        // average all the readings
        double avg_z = z / (double)c;
        uint16_t avg_z_mm = (int)(avg_z * 1000.0);
        depth_avg_img.at<uint16_t>(v, u) = avg_z_mm;
      }
    }
    
    std::string depth_filename = ss_depth_path_.str() + ss_filename_.str();
    cv::imwrite(depth_filename, depth_avg_img);
    ROS_INFO("Depth %s saved", depth_filename.c_str());
    
    cv::imshow("Depth", depth_avg_img);
  }
    
  count_++;  
}

} //namespace ccny_rgbd
