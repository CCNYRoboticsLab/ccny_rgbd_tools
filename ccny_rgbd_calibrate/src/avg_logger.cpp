#include "ccny_rgbd_calibrate/avg_logger.h"

namespace ccny_rgbd {

AvgLogger::AvgLogger(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), nh_private_(nh_private), count_(0), logging_(false),
  rgb_saved_(false)
{
  // **** parameters
  
  if (!nh_private_.getParam ("n_depth", n_depth_))
    n_depth_ = 300;
  if (!nh_private_.getParam ("id", id_))
    id_ = 0;
  if (!nh_private_.getParam ("sequence", sequence_))
    sequence_ = "rgbd";
 
  n_cols_ = 9;
  n_rows_ = 6;

  // ensures directories exist
  prepareDirectories();
  
  // **** subscribers
  
  int queue_size = 5;
  
  ROS_INFO("Subscribing to RGBD");
    
  ImageTransport rgb_it(nh_);
  ImageTransport depth_it(nh_);

  sub_rgb_.subscribe(rgb_it, "/camera/rgb/image_color", 1);
  sub_depth_.subscribe(depth_it, "/camera/depth/image", 1);
  sub_info_.subscribe(nh_, "/camera/rgb/camera_info", 1);
  
  // Synchronize inputs.
  sync_.reset(new RGBDSynchronizer3(
                RGBDSyncPolicy3(queue_size), sub_rgb_, sub_depth_, sub_info_));
  sync_->registerCallback(boost::bind(&AvgLogger::RGBDCallback, this, _1, _2, _3));  
    
  depth_cnt_img_ = cv::Mat::zeros(480, 640, CV_16UC1);
  depth_sum_img_ = cv::Mat::zeros(480, 640, CV_64FC1);
  
  input_thread_ = boost::thread(&AvgLogger::keyboardThread, this);   
}

AvgLogger::~AvgLogger()
{
  ROS_INFO("Destroying RGBD Visual Odometry"); 
}

void AvgLogger::keyboardThread()
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
  cv_bridge::CvImagePtr rgb_ptr = cv_bridge::toCvCopy(rgb_msg);

  // **** display corners (TODO: use the function in calibrate)
  cv::Size pattern_size(n_rows_, n_cols_);
  std::vector<cv::Point2f> corners_2d;
  cv::Mat img_mono;
  cv::cvtColor(rgb_ptr->image, img_mono, CV_RGB2GRAY);
 
  int params = CV_CALIB_CB_ADAPTIVE_THRESH + 
               CV_CALIB_CB_NORMALIZE_IMAGE + 
               CV_CALIB_CB_FAST_CHECK;
  
  bool found = findChessboardCorners(img_mono, pattern_size, corners_2d, params);
  
  if(found)
  {
    cornerSubPix(
      img_mono, corners_2d, pattern_size, cv::Size(-1, -1),
      cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
  
    cv::Mat img_corners = rgb_ptr->image.clone();
    cv::drawChessboardCorners(img_corners, pattern_size, cv::Mat(corners_2d), found);
    cv::imshow("RGB", img_corners);   
  }
  else
  {
    cv::imshow("RGB", rgb_ptr->image);  
  }
  
  cv::waitKey(1);
      
  // if we are not logging, leve here
  if (!logging_) return;

  ROS_INFO("RGBD Callback [%d of %d]", count_+1, n_depth_);
  
  std::stringstream ss_filename;
  ss_filename << std::setw(4) << std::setfill('0') << id_ << ".png";
    
  // accumulate depth images
  if (count_< n_depth_)
  {
    if (found && !rgb_saved_)
    {     
      // write out the current rgb image     
      std::string rgb_filename = ss_rgb_path_.str() + ss_filename.str();
      cv::imwrite(rgb_filename, rgb_ptr->image);
      ROS_INFO("RGB %s saved", rgb_filename.c_str());
      
      rgb_saved_ = true;
    }
    
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
    
    count_++;  
  }
  else if (count_ == n_depth_)
  {
    ROS_INFO("Logging finished");
    
    if (!rgb_saved_)
    {
      // write out the current rgb image     
      std::string rgb_filename = ss_rgb_path_.str() + ss_filename.str();
      cv::imwrite(rgb_filename, rgb_ptr->image);
      ROS_WARN("RGB %s saved, but might be unusable", rgb_filename.c_str());
    }
    
    // create average depth image
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
   
    // reset accumulator andcounter images
    depth_cnt_img_ = cv::Mat::zeros(480, 640, CV_16UC1);
    depth_sum_img_ = cv::Mat::zeros(480, 640, CV_64FC1);
   
    // write out the average depth image
    std::string depth_filename = ss_depth_path_.str() + ss_filename.str();
    cv::imwrite(depth_filename, depth_avg_img);
    ROS_INFO("Depth %s saved", depth_filename.c_str());
      
    // stop logging, bump up image id, and reset counter
    count_ = 0;
    id_++;
    logging_ = false;
    rgb_saved_ = false;
  }
}

} //namespace ccny_rgbd
