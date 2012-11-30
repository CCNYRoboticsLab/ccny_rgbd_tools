/*
 * sphereo.h
 *
 *  Created on: April 12, 2012
 *      Author: Carlos J.
 */

#ifndef REBAGGER_H_
#define REBAGGER_H_

//#include "opencv2/calib3d/calib3d.hpp"
//#include <opencv/highgui.h>
//#include <opencv/cv.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <visualization_msgs/MarkerArray.h>

#include <sensor_msgs/Imu.h>

#include <tf/tfMessage.h>

//#include <sensor_msgs/PointCloud.h>
#include <boost/thread/mutex.hpp>

//#define DEVELOP
//#define REBAGGER_DEVELOP  // Slows down processing since it displays images in OpenCV highgui's window

namespace rgbd_bagger
{
// %EndTag(INCLUDE)%
// %Tag(CLASSDEF)%
// TODO:driver and node are mixed up. They should be separated.
class Rebagger
{
public:
  Rebagger(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  ~Rebagger()
  {
#ifdef DEVELOP
//    cv::destroyWindow(LEFT_WINDOW);
//    cv::destroyWindow(RIGHT_WINDOW);
#endif
  }

private:
  void getParams();
  void controlLoop();
  void processBag(rosbag::Bag *bag_in, rosbag::Bag *bag_out);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  rosbag::Bag *bag_in_;
  rosbag::Bag *bag_out_;

  std::string input_bag_file_name_;
  std::string output_bag_file_name_;
  std::string bag_suffix_;
  int number_of_bags_;

  std::string image_resolution_mode_; ///< indicates either QQVGA=160x120 | QVGA=320x240
  std::string depth_image_encoding_; ///< indicates depth image's cv::Mat type, such as 32FC1, 16UC1, etc.

  std::vector<std::string> topics_in_;

  std::string ground_truth_frame_name_; ///< used for name translation to separate the TF name of the ground truth

  int camera_width_;
  int camera_height_;
  cv::Size output_img_size_;

  double resize_factor_; ///< re-scaling factor for the image optical center and the focal length values
  double depth_resolution_factor_; ///< Value in the depth image that represents 1 meter
  bool loaded_params_; ///< indicates whether parameters from file (as opposed to dynamic) have been loaded

  int interpolation_; ///< interpolation method for remappig (OpenCV) procedure

  bool debug_mode_; ///< useful mode for debugging with a constant image
  bool next_frame_to_debug_; ///< let's capture loop know the desire to switch to a new frame
  bool do_resizing_;

  boost::mutex mutex_lock_; ///< Thread lock on subscribed input images

  // Input bag topic names:
  std::string topic_in_depth_cam_info_;
  std::string topic_in_rgb_cam_info_;
  std::string topic_in_depth_image_;
  std::string topic_in_rgb_image_;
  std::string topic_in_marker_;
  std::string topic_in_imu_;
  std::string topic_in_tf_;
  // Output bag topic names:
  std::string topic_out_depth_cam_info_;
  std::string topic_out_rgb_cam_info_;
  std::string topic_out_depth_image_;
  std::string topic_out_rgb_image_;
  std::string topic_out_marker_;
  std::string topic_out_imu_;
  std::string topic_out_tf_;
};
// %EndTag(CLASSDEF)%



} // end namespace

#endif /* REBAGGER_H_ */
