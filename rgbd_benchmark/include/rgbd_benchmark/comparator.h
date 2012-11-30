/*
 * comparator.h
 *
 *  Created on: Sep 13, 2011
 *      Author: carlos
 */

#ifndef COMPARATOR_H_
#define COMPARATOR_H_

#include <ros/ros.h>
#include <std_msgs/Time.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>

#define DEVELOP_TF

namespace rgbd_benchmark
{
const std::string trigger_topic = "/dummy"; ///< Name of topic to subscribe to

class Comparator {
public:
  Comparator(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
  ~Comparator()
  {
    if(save_to_file_)
      {
      fclose(pFile_);
      ROS_INFO("Closed file %s", error_file_name_.c_str());
      }
  };

  void broadcastTF(tf::Transform &f2b_, const std::string &parent_frame, const std::string &child_frame, const ros::Time &time = ros::Time::now());

private:

  void compareTFCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg_ptr);     ///< Callback handler for error benchmark comparator
  void getParams();

  /*
   * @brief Saves computed error information to file (or print on screen)
   * @param error_magnitude The magnitude of error
   * @param angular_error Directional error
   * @param stamp Time stamp for frames used in computation
   * @param mode  false: print to screen (default).  true:save to file)
   */
  void collectError(double error_magnitude, double angular_error, ros::Time &stamp, bool mode);

  /**
   * @brief Saves computed error information to file (or print on screen)
   * @param file_name File name where to write error data for further plotting/analysis
   * @param error_magnitude The magnitude of error
   * @param angular_error Directional error
   * @param stamp Time stamp for frames used in computation
   * @return 1 if write to file was successful
   */
  int saveErrorToFile(double error_magnitude, double angular_error, ros::Duration &stamp);

  /**
   * @brief The inverse motion composition operator
   * @param a current frame
   * @param b previous frame
   */
  tf::Transform ominus(tf::Transform &a, tf::Transform &b);

  FILE * pFile_;

  ros::Time init_time_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher pub_m;
  ros::Publisher pub_a;

  std::string fixed_frame_gt_; ///< Fixed reference frame id for ground truth position
  std::string fixed_frame_est_; ///< Fixed reference frame id for estimated position
  std::string target_frame_gt_; /// Target's frame id of the ground truth data (e.g. camera_link, base_link, etc)
  std::string target_frame_est_; /// Target's frame id of the estimated data (e.g. camera_link, base_link, etc)

  std::string error_file_name_;  ///< File name for error information retrieval
  bool save_to_file_; ///< indicates whether to save results to file or print to screen
  bool has_tf_gt_; ///< to know when we got the transform of the ground truth frame
  bool has_tf_est_; ///< to know when we got the transform of the estimated frame

  // TODO: when fanciness allows
//  message_filters::Subscriber<std_msgs::Time>* dummy_msg_filter_sub_fancy_; ///< It simply alerts the tf listener for the availability of new transformations
//  tf::MessageFilter<std_msgs::Time>* dummy_tf_filter_;

  ros::Subscriber pose_msg_sub_; ///< It simply alerts the tf listener for the availability of new transformations

  tf::TransformListener    tf_listener_;
  tf::StampedTransform previous_fixed2target_tf_gt_;
  tf::StampedTransform previous_fixed2target_tf_est_;

  tf::TransformBroadcaster tf_broadcaster_;
  tf::StampedTransform gt2est_tf_init_;  ///< Initial transform between ground_truth and estimation frames
  tf::StampedTransform gt2est_tf_;  ///< Transformed estimation frame to ground truth (good for visualization) as the init error has been taken into account.

  double magnitude_error_accumulator_;
  double angular_error_accumulator_;
};

} // end namespace

#endif /* COMPARATOR_H_ */
