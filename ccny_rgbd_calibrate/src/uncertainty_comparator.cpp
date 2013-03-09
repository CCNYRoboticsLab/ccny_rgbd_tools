#include "ccny_rgbd_calibrate/uncertainty_comparator.h"

namespace ccny_rgbd {

UncertaintyLogger::UncertaintyLogger(
  const ros::NodeHandle& nh, 
  const ros::NodeHandle& nh_private):
  nh_(nh), nh_private_(nh_private), count_(0), logging_(false),
  rgb_saved_(false)
{
  // **** parameters
  
  if (!nh_private_.getParam ("n_images", n_depth_))
    n_depth_ = 300;
  if (!nh_private_.getParam ("id", id_))
    id_ = 0;
  if (!nh_private_.getParam ("n_cols", n_cols_))
    n_cols_ = 8;
  if (!nh_private_.getParam ("n_rows", n_rows_))
    n_rows_ = 6;
  if (!nh_private_.getParam ("path", path_))
    ROS_ERROR("path param needs to be set");
  
  // ensures directories exist
  prepareDirectories();
  
  // **** subscribers
  
  int queue_size = 5;
  
  ROS_INFO("Subscribing to RGBD");
    
  ImageTransport rgb_it(nh_);
  ImageTransport depth_it(nh_);

  sub_rgb_.subscribe(rgb_it, "/camera/rgb/image_color", 1);
  sub_depth_.subscribe(depth_it, "/camera/depth/image_raw", 1);
  sub_info_.subscribe(nh_, "/camera/rgb/camera_info", 1);
  
  // Synchronize inputs.
  sync_.reset(new RGBDSynchronizer3(
                RGBDSyncPolicy3(queue_size), sub_rgb_, sub_depth_, sub_info_));
  sync_->registerCallback(boost::bind(&AvgLogger::RGBDCallback, this, _1, _2, _3));  
    
  // http://www.johndcook.com/standard_deviation.html
  c_img_ = cv::Mat::zeros(480, 640, CV_16UC1);  // conter image
  m_img_ = cv::Mat::zeros(480, 640, CV_64FC1);  // mean accumulator
  s_img_ = cv::Mat::zeros(480, 640, CV_64FC1);  // stdev accumulator
  
  input_thread_ = boost::thread(&AvgLogger::keyboardThread, this);   
}

UncertaintyLogger::~UncertaintyLogger()
{
  ROS_INFO("Destroying RGBD Visual Odometry"); 
}

void UncertaintyLogger::keyboardThread()
{
  ros::Rate r(20);
  
  while(true)
  {
    if(!logging_)
    {
      ROS_INFO("Press [Enter] to start logging image %d", id_);
      getchar();
      ROS_INFO("Logging enabled");
      logging_ = true;
    }
    
    r.sleep();
  }
}

void UncertaintyLogger::prepareDirectories()
{ 
  ROS_INFO("Creating directory: %s", path_.c_str());
  boost::filesystem::create_directory(path_);
  
  rgb_path_   = path_ + "/rgb/";
  depth_path_ = path_ + "/depth/";
  stdev_path_ = path_ + "/stdev/";
  stdev_q_path_ = path_ + "/stdev_q/";
  stdev_qgmm_path_ = path_ + "/stdev_qgmm/";

  boost::filesystem::create_directory(rgb_path_); 
  boost::filesystem::create_directory(depth_path_); 
  boost::filesystem::create_directory(stdev_path_); 
  boost::filesystem::create_directory(stdev_q_path_); 
  boost::filesystem::create_directory(stdev_qgmm_path_); 
}

void UncertaintyLogger::RGBDCallback(
  const sensor_msgs::ImageConstPtr& rgb_msg,
  const sensor_msgs::ImageConstPtr& depth_msg,
  const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(rgb_msg);
  cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg);

  cv::Mat& rgb_img   = rgb_ptr->image;
  cv::Mat& depth_img = depth_ptr->image;
     
  // **** display corners (TODO: use the function in calibrate)
  cv::imshow("RGB", rgb_img);  
  cv::waitKey(1);
      
  // if we are not logging, leave here
  if (!logging_) return;

  ROS_INFO("RGBD Callback [%d of %d]", count_+1, n_depth_);
  
  std::stringstream ss_filename;
  ss_filename << std::setw(4) << std::setfill('0') << id_ << ".png";
    
  // accumulate depth images
  if (count_< n_depth_)
  {  
    for (int u = 0; u < depth_img.cols; ++u)
    for (int v = 0; v < depth_img.rows; ++v)
    {
      float z = depth_img.at<uint16_t>(v, u);
      
      if (!isnan(z))
      {
        // increment counter
        c_img_.at<uint16_t>(v, u) += 1;
        int c = c_img_.at<uint16_t>(v, u);
        
        // running mean and stdev 
        double old_m = m_img_.at<double>(v, u);
        double new_m = old_m + (z - old_m) / (double)c;
        
        double old_s = s_img_.at<double>(v, u);
        double new_s = old_s + (z - old_m) * (z - new_m);
        
        m_img_.at<double>(v, u) = new_m;
        s_img_.at<double>(v, u) = new_s;      
      }
    }
    
    count_++;  
  }
  else if (count_ == n_depth_)
  {
    ROS_INFO("Logging finished");
       
    // write out the current rgb image     
    std::string rgb_filename = rgb_path_+ ss_filename.str();
    cv::imwrite(rgb_filename, rgb_ptr->image);
    ROS_WARN("RGB image saved to %s", rgb_filename.c_str());
    
    // create average depth image
    cv::Mat depth_mean_img_uint = cv::Mat::zeros(480, 640, CV_16UC1);
    m_img_.convertTo(depth_mean_img_uint, CV_16UC1);
  
    std::string depth_filename = depth_path_ + ss_filename.str();
    cv::imwrite(depth_filename, depth_mean_img_uint);
    ROS_INFO("Depth image saved to %s", depth_filename.c_str());
    
    // create the stdev depth image (in nm)
    cv::Mat depth_std_img = cv::Mat::zeros(480, 640, CV_32FC1);
    
    for (int u = 0; u < depth_std_img.cols; ++u)
    for (int v = 0; v < depth_std_img.rows; ++v)
      stdev_gt_img_.at<float>(v, u) = getStDev(v, u);
    
    std::string stdev_filename = stdev_path_ + ss_filename.str();
    saveUncertaintyImage(depth_std_img, stdev_filename);
    ROS_INFO("Stdev image saved to %s", stdev_filename.c_str());
    
    // build pred unc. images and process them 
    buildUncertaintyImages();
    
    // reset accumulator and counter images
    c_img_ = cv::Mat::zeros(480, 640, CV_16UC1);
    m_img_ = cv::Mat::zeros(480, 640, CV_64FC1);
    s_img_ = cv::Mat::zeros(480, 640, CV_64FC1);
      
    // stop logging, bump up image id, and reset counter
    count_ = 0;
    id_++;
    logging_ = false;
  }
}

void UncertaintyLogger::buildUncertaintyImages(
  const cv::Mat& depth_img)
{
  
  
}

double UncertaintyLogger::getStDev(int v, int u)
{
  int c = c_img_.at<uint16_t>(v, u);
  
  double std_dev_z;
  if (c > 1)
  {
    double s = s_img_.at<double>(v, u);
    double var_z = s / (double)(c - 1);
    std_dev_z = sqrt(var_z);
  }
  else
    std_dev_z = 0.0;
  
  return std_dev_z;
}

} // namespace ccny_rgbd
