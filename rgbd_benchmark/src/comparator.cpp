/*
 * comparator.cpp
 *
 *  Created on: Sep 13, 2011
 *      Author: carlos
 */

#include <rgbd_benchmark/comparator.h>

namespace rgbd_benchmark
{

Comparator::Comparator(ros::NodeHandle &nh, ros::NodeHandle &nh_private) :
  nh_(nh), nh_private_(nh_private)
{
  getParams();

  if(save_to_file_)
    pFile_ = fopen(error_file_name_.c_str(), "w");

  // dynamic reconfigure:
  /*
   // -------------------------------
   dynamic_reconfigure::Server<CatasphereoParamsConfig>::CallbackType f =
   boost::bind(&Sphereo::reconfig_callback, this, _1, _2);
   dyn_recfg_srv_.setCallback(f);
   // ---------------------------------------------------------
   stereo_params_change_ = true;
   */

  // Initialize error accumulators
  magnitude_error_accumulator_ = 0.0;
  angular_error_accumulator_ = 0.0;

  // %Tag(SUB)%
//  dummy_msg_sub_ = nh_.subscribe("dummy", 1000, &Comparator::compareTFCallback, this);
  pose_msg_sub_ = nh_.subscribe(trigger_topic, 1000, &Comparator::compareTFCallback, this);
  // %EndTag(SUB)%

  pub_m = nh_.advertise<std_msgs::Float32>("error_m", 1);
  pub_a = nh_.advertise<std_msgs::Float32>("error_a", 1);

  // TODO: fancier way of subscribing (for the future)
//  	dummy_msg_filter_sub_fancy_ = new message_filters::Subscriber<std_msgs::Time>(nh_, trigger_topic, 5);
//    dummy_tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(*dummy_msg_filter_sub_fancy_, tf_listener_, odom_frame_, 5);
//    dummy_tf_filter_->registerCallback(boost::bind(&Comparator::compareTFCallback, this, _1));
}

// %Tag(CALLBACK)%
void Comparator::compareTFCallback(const geometry_msgs::PoseStampedConstPtr& pose_msg_ptr)
{
  ros::Time time_stamp = pose_msg_ptr->header.stamp;
#ifdef DEVELOP_TF
//  ROS_INFO("Subscribing to topic with frame_id=%s, Pose Estimation at ($.2%f, $.2%f, $.2%f)", pose_msg_ptr->header.frame_id.c_str(), pose_msg_ptr->pose.position.x, pose_msg_ptr->pose.position.y, pose_msg_ptr->pose.position.z);
#endif

  // ------- get odom to kinect transform (tf) -----------------------
  tf::StampedTransform current_tf_gt;
  tf::StampedTransform current_tf_est;

  try
  {
    has_tf_gt_ = tf_listener_.waitForTransform(fixed_frame_gt_, target_frame_gt_, time_stamp, ros::Duration(0.02));
    tf_listener_.lookupTransform (fixed_frame_gt_, target_frame_gt_, time_stamp, current_tf_gt);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN( "Transform error from %s to %s, quitting.", 
      fixed_frame_gt_.c_str(), target_frame_gt_.c_str());
    return;
  }
  try
  {
    has_tf_est_ = tf_listener_.waitForTransform(fixed_frame_est_, target_frame_est_, time_stamp, ros::Duration(1.0));
    tf_listener_.lookupTransform(fixed_frame_est_, target_frame_est_, time_stamp, current_tf_est);
  }
  catch (tf::TransformException& ex)
  {
    ROS_WARN( "Transform error from %s to %s, quitting.", 
      fixed_frame_est_.c_str(), target_frame_est_.c_str());
    return;
  }

  static bool is_first_time = true;

  if (is_first_time)
  {
    // TODO: store 0 as error

    init_time_ = time_stamp;

    if(has_tf_gt_ && has_tf_est_)
    {
      // Compute transform from fixed gt and est
      gt2est_tf_init_.setData(current_tf_gt * current_tf_est.inverse()); // Correct estimation transformation to broadcast
      broadcastTF(gt2est_tf_init_, fixed_frame_gt_, fixed_frame_est_, time_stamp); // FIXME: frame ids
#ifdef DEVELOP_TF
      std::cout << std::endl << "Initial Ground Truth to Estimation transform at ("
                << gt2est_tf_init_.getOrigin().getX() << ","
                << gt2est_tf_init_.getOrigin().getY() << ","
                << gt2est_tf_init_.getOrigin().getZ() << ","
                << ")"<< std::endl;

      btQuaternion q = current_tf_gt.getRotation().asBt();
      double roll, pitch, yaw;
      btMatrix3x3(q).getEulerYPR(yaw, pitch, roll);
      ROS_INFO("GT Fixed Frame's YPR = (%lf, %lf, %lf)", yaw, pitch, roll);
#endif
      gt2est_tf_.setData(gt2est_tf_init_ * current_tf_est);

      is_first_time = false;
    }
  }
  else if(has_tf_gt_ && has_tf_est_)
  {
//    gt2est_tf_.setData(current_tf_est * gt2est_tf_init_.inverse()); // Correct estimation transformation to broadcast
    gt2est_tf_.setData(gt2est_tf_init_ * current_tf_est);
    tf::Transform motion_comp_est  = ominus(gt2est_tf_,  previous_fixed2target_tf_est_);
    tf::Transform motion_comp_gt = ominus(current_tf_gt, previous_fixed2target_tf_gt_);
    tf::Transform error44_tf = ominus(motion_comp_est, motion_comp_gt);

    double error_magnitude = error44_tf.getOrigin().length();
    magnitude_error_accumulator_ += error_magnitude;
    
    float trace = error44_tf.getBasis()[0][0] + error44_tf.getBasis()[1][1] + error44_tf.getBasis()[2][2];
    double angular_error = acos(std::min(1.0, std::max(-1.0, (trace - 1.0)/2.0))) * (180.0/M_PI);
    
    //btVector3 u(1,0,0);
    //double angular_error = u.dot(error44_tf * u);

    angular_error_accumulator_ += angular_error;

    // save results to file (or screen printout...mode 0)
    collectError(magnitude_error_accumulator_, angular_error_accumulator_, time_stamp, save_to_file_);
  } 

  previous_fixed2target_tf_gt_ = current_tf_gt;
  previous_fixed2target_tf_est_  = gt2est_tf_;

  broadcastTF(gt2est_tf_init_, fixed_frame_gt_, fixed_frame_est_, time_stamp); // FIXME: frame ids

}
// %EndTag(CALLBACK)%

tf::Transform Comparator::ominus(tf::Transform &a, tf::Transform &b)
{
	tf::Transform b_inv = b.inverse();
	return b_inv * a;
}
// %Tag(PARAMS)%
void Comparator::getParams()
{
  // Frame Transforms:
  if (!nh_private_.getParam("fixed_frame_gt", fixed_frame_gt_))
    fixed_frame_gt_ = "world";
  if (!nh_private_.getParam("fixed_frame_est", fixed_frame_est_))
    fixed_frame_est_ = "odom";
  if (!nh_private_.getParam("target_frame_gt", target_frame_gt_))
    target_frame_gt_ = "target_gt";
  if (!nh_private_.getParam("target_frame_est", target_frame_est_))
    target_frame_est_ = "target_est";

  // File I/O
  if (!nh_private_.getParam("error_file_name", error_file_name_))
    error_file_name_ = "benchmark_error";
  if (!nh_private_.getParam("save_to_file", save_to_file_))
    save_to_file_ = false;

}
// %EndTag(PARAMS)%


void Comparator::broadcastTF(tf::Transform &f2b_, const std::string &parent_frame, const std::string &child_frame, const ros::Time &timestamp)
{
  tf::StampedTransform transform_msg(f2b_, timestamp, parent_frame, child_frame);
  tf_broadcaster_.sendTransform (transform_msg);

//  geometry_msgs::PoseStamped pose_msg;
//  tf::poseTFToMsg(f2b_, pose_msg.pose);
//  pose_msg.header = header;
//  pose_pub_.publish(pose_msg);
}
/*
 * @brief Saves computed error information to file (or print on screen)
 * @param error_magnitude The magnitude of error
 * @param angular_error Directional error
 * @param stamp Time stamp for frames used in computation
 * @param mode  false: print to screen (default).  true:save to file)
 */
void Comparator::collectError(double error_magnitude, double angular_error, ros::Time &stamp, bool mode)
{
  ros::Duration t = stamp - init_time_;

  //   if(g_bValidGroundPlane)
  if (mode)
    saveErrorToFile(error_magnitude, angular_error, t); // Write error info to file

  ROS_INFO("[%f] Mag: %f, Angle: %f", t.toSec(), error_magnitude, angular_error);

  std_msgs::Float32 m, a;
  m.data = error_magnitude;  
  a.data = angular_error;  
  pub_m.publish(m);
  pub_a.publish(a);
}

/*
 * @brief Saves computed error information to file (or print on screen)
 * @param file_name File name where to write error data for further plotting/analysis
 * @param error_magnitude The magnitude of error
 * @param angular_error Directional error
 * @param stamp Time stamp for frames used in computation
 * @return 1 if write to file was successful
 */
int Comparator::saveErrorToFile(double error_magnitude, double angular_error, ros::Duration &stamp)
{
  if (pFile_ == NULL)
  {
    printf("saveErrorToFile: Can't create %s\n", error_file_name_.c_str());
    return false;
  }

  fprintf(pFile_, "%f,%f,%f\n", stamp.toSec(), error_magnitude, angular_error);

  return 0; 
}

}// end namespace
