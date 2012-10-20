#include "ccny_rgbd/apps/logger.h"

namespace ccny_rgbd {

Logger::Logger(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), nh_private_(nh_private), id_(0)
{
  // **** subscribers

  image_transport::ImageTransport rgb_it(nh_);
  image_transport::ImageTransport depth_it(nh_);

  sub_depth_.subscribe(
    depth_it, "/camera/depth_registered/image_rect_raw", 1);  // uint16_t, in mm - opencv doesn't save floats
  sub_rgb_.subscribe(
    rgb_it, "/camera/rgb/image_rect_color", 1);
  sub_info_.subscribe(
    nh_, "/camera/rgb/camera_info", 1);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  int queue_size = 5;
  sync_.reset(new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_));
  sync_->registerCallback(boost::bind(&Logger::RGBDCallback, this, _1, _2, _3));  

  if (!nh_private_.getParam ("rgb", rgb_))
    rgb_ = true;
  if (!nh_private_.getParam ("depth", depth_))
    depth_ = true;
  if (!nh_private_.getParam ("sequence", sequence_))
    sequence_ = "rgbd";
  if (!nh_private_.getParam ("n", n_))
    n_ = 1000;

  // prepare directories
  prepareDirectories();
}

Logger::~Logger()
{
  ROS_INFO("Destroying RGBD Visual Odometry"); 
}

void Logger::prepareDirectories()
{
  std::stringstream ss_seq_path;
  ss_seq_path << "/home/idryanov/ros/images/" << sequence_;
  ss_rgb_path_ << ss_seq_path.str() << "/rgb/";
  ss_depth_path_ << ss_seq_path.str() << "/depth/";

  ROS_INFO("creating directory %s", ss_seq_path.str().c_str());
  boost::filesystem::create_directory(ss_seq_path.str());

  boost::filesystem::create_directory(ss_rgb_path_.str()); 
  boost::filesystem::create_directory(ss_depth_path_.str()); 
}

void Logger::RGBDCallback(
  const sensor_msgs::ImageConstPtr& depth_msg,
  const sensor_msgs::ImageConstPtr& rgb_msg,
  const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if (id_< n_)
  {
    std::stringstream ss_filename;
    ss_filename << std::setw(4) << std::setfill('0') << id_ << ".png";

    cv_bridge::CvImagePtr rgb_ptr   = cv_bridge::toCvCopy(rgb_msg);
    cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg);

    if (rgb_)
      cv::imwrite(ss_rgb_path_.str()   + ss_filename.str(), rgb_ptr->image);
    if (depth_)
      cv::imwrite(ss_depth_path_.str() + ss_filename.str(), depth_ptr->image);

    ROS_INFO("RGBD %s saved", ss_filename.str().c_str());

/*
    cv::namedWindow("RGB", 1);
    cv::imshow("RGB", rgb_ptr->image);
    cv::waitKey(1);
    cv::namedWindow("Depth", 1);
    cv::imshow("Depth", depth_ptr->image);
*/

    id_++;
  }
}


} //namespace ccny_rgbd
