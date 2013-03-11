#include "ccny_rgbd_calibrate/uncertainty_comparator.h"

namespace ccny_rgbd {

UncertaintyComparator::UncertaintyComparator(
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
  sync_->registerCallback(boost::bind(&UncertaintyComparator::RGBDCallback, this, _1, _2, _3));  
    
  // http://www.johndcook.com/standard_deviation.html
  c_img_ = cv::Mat::zeros(480, 640, CV_16UC1);  // conter image
  m_img_ = cv::Mat::zeros(480, 640, CV_64FC1);  // mean accumulator
  s_img_ = cv::Mat::zeros(480, 640, CV_64FC1);  // stdev accumulator
  
  input_thread_ = boost::thread(&UncertaintyComparator::keyboardThread, this);   
}

UncertaintyComparator::~UncertaintyComparator()
{
  ROS_INFO("Destroying RGBD Visual Odometry"); 
}

void UncertaintyComparator::keyboardThread()
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

void UncertaintyComparator::prepareDirectories()
{ 
  ROS_INFO("Creating directory: %s", path_.c_str());
  boost::filesystem::create_directory(path_);
  
  rgb_path_        = path_ + "/rgb/";
  depth_gt_path_   = path_ + "/depth_gt/";
  depth_test_path_ = path_ + "/depth_tr/";
  
  stdev_gt_path_   = path_ + "/stdev_gt/";
  stdev_q_path_    = path_ + "/stdev_q/";
  stdev_qgmm_path_ = path_ + "/stdev_qgmm/";

  boost::filesystem::create_directory(rgb_path_); 
  boost::filesystem::create_directory(depth_gt_path_); 
  boost::filesystem::create_directory(stdev_gt_path_); 
  boost::filesystem::create_directory(stdev_q_path_); 
  boost::filesystem::create_directory(stdev_qgmm_path_); 
}

void UncertaintyComparator::RGBDCallback(
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
    cv::imwrite(rgb_filename, rgb_img);
    ROS_WARN("RGB image saved to %s", rgb_filename.c_str());
    
    // write out the current depth image, for use as test image
    depth_test_img_ = depth_img.clone();
    
    std::string depth_test_filename = depth_test_path_+ ss_filename.str();
    cv::imwrite(depth_test_filename, depth_test_img_);
    ROS_INFO("Depth_test image saved to %s", depth_test_filename.c_str());
    
    // create average depth image, for use as ground truth image
    m_img_.convertTo(depth_gt_img_, CV_16UC1);
  
    std::string depth_gt_filename = depth_gt_path_ + ss_filename.str();
    cv::imwrite(depth_gt_filename, depth_gt_img_);
    ROS_INFO("Depth image saved to %s", depth_gt_filename.c_str());
    
    // create the ground truth stdev image (in nm)    
    for (int u = 0; u < s_img_.cols; ++u)
    for (int v = 0; v < s_img_.rows; ++v)
      stdev_gt_img_.at<float>(v, u) = getStDev(v, u);
    
    std::string stdev_gt_filename = stdev_gt_path_ + ss_filename.str();
    saveUncertaintyImage(stdev_gt_img_, stdev_gt_filename);
    ROS_INFO("Stdev ground truth image saved to %s", stdev_gt_filename.c_str());
    
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

void UncertaintyComparator::buildUncertaintyImages()
{
  
  
}

double UncertaintyComparator::getStDev(int v, int u)
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
