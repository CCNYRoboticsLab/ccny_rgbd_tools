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

  sub_rgb_.subscribe(rgb_it, "/rgbd/rgb", queue_size);
  sub_depth_.subscribe(depth_it, "/rgbd/depth", queue_size);
  sub_info_.subscribe(nh_, "/rgbd/info", queue_size);
  
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
  
  rgb_test_path_   = path_ + "/rgb/";
  depth_gt_path_   = path_ + "/depth_gt/";
  depth_test_path_ = path_ + "/depth_tr/";
  
  stdev_gt_path_   = path_ + "/stdev_gt/";
  stdev_q_path_    = path_ + "/stdev_q/";
  stdev_qgmm_path_ = path_ + "/stdev_qgmm/";

  boost::filesystem::create_directory(rgb_test_path_); 
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
  cv_bridge::CvImageConstPtr rgb_ptr = cv_bridge::toCvShare(rgb_msg);
  cv_bridge::CvImageConstPtr depth_ptr = cv_bridge::toCvShare(depth_msg);

  const cv::Mat& rgb_img   = rgb_ptr->image;
  const cv::Mat& depth_img = depth_ptr->image;
     
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
      uint16_t z = depth_img.at<uint16_t>(v, u);
      
      if (z != 0)
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
    rgb_test_img_ = rgb_img.clone();
    
    std::string rgb_test_filename = rgb_test_path_+ ss_filename.str();
    cv::imwrite(rgb_test_filename, rgb_test_img_);
    
    // create average depth image, for use as ground truth image
    m_img_.convertTo(depth_gt_img_, CV_16UC1);
  
    std::string depth_gt_filename = depth_gt_path_ + ss_filename.str();
    cv::imwrite(depth_gt_filename, depth_gt_img_);

    // write out the current depth image, for use as test image
    depth_test_img_ = depth_img.clone();
    //depth_test_img_ = depth_gt_img_.clone();
    
    std::string depth_test_filename = depth_test_path_+ ss_filename.str();
    cv::imwrite(depth_test_filename, depth_test_img_);
        
    // build pred unc. images and process them 
    
    buildStDevGroundTruthImage();
    std::string stdev_gt_filename = stdev_gt_path_ + ss_filename.str();
    saveUncertaintyImage(stdev_gt_img_, stdev_gt_filename);
    
    buildStDevQuadraticImage();    
    std::string stdev_q_filename = stdev_q_path_ + ss_filename.str();
    saveUncertaintyImage(stdev_q_img_, stdev_q_filename);

    // eval

    double rms_q, rms_f_q;

    std::string stdev_qgmm_filename = stdev_qgmm_path_ + ss_filename.str();
    
    std::string rms_filename = path_ + "/error_" + ss_filename.str() + ".txt";
    cv::FileStorage fs(rms_filename, cv::FileStorage::WRITE);

    // SPAGHETTI, SPAGHETTI
    // 0 ---------------------------------

    double rms_qgmm_0, rms_f_qgmm_0;
    w0_ = 4.0; w1_ = 2.0; w2_ = 1.0;
    buildStDevQuadraticGMMImage();
    saveUncertaintyImage(stdev_qgmm_img_, stdev_qgmm_filename);
        
    evaluateRMSError(rms_q, rms_qgmm_0);
    evaluateRMSErrorFeatures(rms_f_q, rms_f_qgmm_0);
    
    fs << "rms_q" << rms_q;
    fs << "rms_f_q" << rms_f_q;
    fs << "rms_qgmm_0" << rms_qgmm_0;
    fs << "rms_f_qgmm_0" << rms_f_qgmm_0;
    
    // 1 ---------------------------------

    double rms_qgmm_1, rms_f_qgmm_1;
    w0_ = 10.0; w1_ = 3.0; w2_ = 1.0;
    buildStDevQuadraticGMMImage();
    saveUncertaintyImage(stdev_qgmm_img_, stdev_qgmm_filename);
        
    evaluateRMSError(rms_q, rms_qgmm_1);
    evaluateRMSErrorFeatures(rms_f_q, rms_f_qgmm_1);
    
    fs << "rms_qgmm_1" << rms_qgmm_1;
    fs << "rms_f_qgmm_1" << rms_f_qgmm_1;
    
    // 2 ---------------------------------

    double rms_qgmm_2, rms_f_qgmm_2;
    w0_ = 4.0; w1_ = 1.0; w2_ = 0.0;
    buildStDevQuadraticGMMImage();
    saveUncertaintyImage(stdev_qgmm_img_, stdev_qgmm_filename);
        
    evaluateRMSError(rms_q, rms_qgmm_2);
    evaluateRMSErrorFeatures(rms_f_q, rms_f_qgmm_2);
    
    fs << "rms_qgmm_2" << rms_qgmm_2;
    fs << "rms_f_qgmm_2" << rms_f_qgmm_2;
    
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
  buildStDevGroundTruthImage();
  buildStDevQuadraticImage();
  buildStDevQuadraticGMMImage();
}

void UncertaintyComparator::evaluateRMSError(
  double& rms_q, double& rms_qgmm)
{
  double sum_q    = 0.0;
  double sum_qgmm = 0.0;
  
  double counter = 0.0;
  
  for (int u = 0; u < stdev_gt_img_.cols; ++u)
  for (int v = 0; v < stdev_gt_img_.rows; ++v)
  {
    uint16_t z = depth_test_img_.at<uint16_t>(v, u);
    
    if (z != 0)
    {
      double stdev_gt   = stdev_gt_img_.at<double>(v, u);
      double stdev_q    = stdev_q_img_.at<double>(v, u);
      double stdev_qgmm = stdev_qgmm_img_.at<double>(v, u);
      
      double dq    = stdev_gt - stdev_q;
      double dqgmm = stdev_gt - stdev_qgmm;
      
      sum_q    += dq * dq;
      sum_qgmm += dqgmm * dqgmm;
      
      counter += 1.0;
    }
  }
  
  rms_q    = sqrt(sum_q    / counter);
  rms_qgmm = sqrt(sum_qgmm / counter);

  printf("Total: RMS_Q: %f, RMS_QGMM: %f\n", rms_q, rms_qgmm);
}

void UncertaintyComparator::evaluateRMSErrorFeatures(
  double& rms_q, double& rms_qgmm)
{
  cv::GoodFeaturesToTrackDetector gft(300, 0.01, 1.0);
  cv::Mat mask(depth_test_img_.size(), CV_8UC1);
  depth_test_img_.convertTo(mask, CV_8U);

  KeypointVector keypoints;
  gft.detect(rgb_test_img_, keypoints, mask);

  double sum_q    = 0.0;
  double sum_qgmm = 0.0;
  
  double counter = 0.0;
  
  for (int idx = 0; idx < keypoints.size(); ++idx)
  {
    const cv::KeyPoint& kp = keypoints[idx];
    
    int u = kp.pt.x;
    int v = kp.pt.y;

    uint16_t z = depth_test_img_.at<uint16_t>(v, u);
    
    if (z != 0)
    {
      double stdev_gt   = stdev_gt_img_.at<double>(v, u);
      double stdev_q    = stdev_q_img_.at<double>(v, u);
      double stdev_qgmm = stdev_qgmm_img_.at<double>(v, u);
      
      double dq    = stdev_gt - stdev_q;
      double dqgmm = stdev_gt - stdev_qgmm;
      
      sum_q    += dq * dq;
      sum_qgmm += dqgmm * dqgmm;
      
      counter += 1.0;
    }
  }
  
  rms_q    = sqrt(sum_q    / counter);
  rms_qgmm = sqrt(sum_qgmm / counter);
  
  printf("Features: RMS_Q: %f, RMS_QGMM: %f\n", rms_q, rms_qgmm);
}

void UncertaintyComparator::buildStDevGroundTruthImage()
{
  stdev_gt_img_ = cv::Mat(depth_gt_img_.size(), CV_64FC1);
      
  for (int u = 0; u < s_img_.cols; ++u)
  for (int v = 0; v < s_img_.rows; ++v)
  {
    double stdev_mm = getStDevGT(v, u);

    stdev_gt_img_.at<double>(v, u) = stdev_mm; 
  }
}

void UncertaintyComparator::buildStDevQuadraticImage()
{
  stdev_q_img_ = cv::Mat(depth_gt_img_.size(), CV_64FC1);
  
  for (int u = 0; u < stdev_q_img_.cols; ++u)
  for (int v = 0; v < stdev_q_img_.rows; ++v)
  {
    uint16_t z_mm = depth_gt_img_.at<uint16_t>(v, u);

    double stdev_mm;
    if (z_mm != 0)
    {
      double z_m = z_mm * 0.001; 
      double stdev_m = 0.001425 * z_m * z_m;
      stdev_mm = stdev_m * 1000.0;
    }
    else stdev_mm = 0.0;
          
    stdev_q_img_.at<double>(v, u) = stdev_mm;
  }
}

void UncertaintyComparator::buildStDevQuadraticGMMImage()
{
  stdev_qgmm_img_ = cv::Mat(depth_gt_img_.size(), CV_64FC1);
  
  for (int u = 0; u < stdev_q_img_.cols; ++u)
  for (int v = 0; v < stdev_q_img_.rows; ++v)
  {
    uint16_t z_mm = depth_gt_img_.at<uint16_t>(v, u);

    double stdev_mm;
    if (z_mm != 0)
    {
      double mean_m, var_m;
      getGaussianMixtureDistribution(u, v, mean_m, var_m);
      double stdev_m = sqrt(var_m);
      stdev_mm = stdev_m * 1000.0;
    }
    else stdev_mm = 0.0;
    
    stdev_qgmm_img_.at<double>(v, u) = stdev_mm;
  }
}

void UncertaintyComparator::getGaussianMixtureDistribution(
  int u, int v, double& z_mean, double& z_var)
{
  /// @todo Different window sizes? based on sigma_u, sigma_v?
  int w = 1;

  int u_start = std::max(u - w, 0);
  int v_start = std::max(v - w, 0);
  int u_end   = std::min(u + w, depth_gt_img_.cols - 1);
  int v_end   = std::min(v + w, depth_gt_img_.rows - 1);

  // iterate accross window - find mean
  double weight_sum = 0.0;
  double mean_sum   = 0.0;
  double alpha_sum  = 0.0;

  for (int uu = u_start; uu <= u_end; ++uu)
  for (int vv = v_start; vv <= v_end; ++vv)
  {
    uint16_t z_neighbor_raw = depth_gt_img_.at<uint16_t>(vv, uu);
 
    if (z_neighbor_raw != 0)
    {
      double z_neighbor = z_neighbor_raw * 0.001;

      // determine and aggregate weight
      double weight;
      if       (u==uu && v==vv) weight = w0_;
      else if  (u==uu || v==vv) weight = w1_;
      else                      weight = w2_; 
      weight_sum += weight;

      // aggregate mean
      mean_sum += weight * z_neighbor;

      // aggregate var
      double stdev_z_neighbor = 0.001425 * z_neighbor * z_neighbor;
      double var_z_neighbor = stdev_z_neighbor * stdev_z_neighbor;
      alpha_sum += weight * (var_z_neighbor + z_neighbor * z_neighbor);
    }
  }

  z_mean = mean_sum  / weight_sum;
  z_var  = alpha_sum / weight_sum - z_mean * z_mean;
}

double UncertaintyComparator::getStDevGT(int v, int u)
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
