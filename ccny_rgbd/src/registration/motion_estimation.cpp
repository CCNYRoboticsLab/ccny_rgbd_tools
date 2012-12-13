#include "ccny_rgbd/registration/motion_estimation.h"

namespace ccny_rgbd
{

MotionEstimation::MotionEstimation(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  have_odom_msg_(false)
{
  // params

  if (!nh_private_.getParam ("reg/min_feature_count", min_feature_count_))
    min_feature_count_ = 15;
  if (!nh_private_.getParam ("reg/motion_constraint", motion_constraint_ ))
    motion_constraint_  = 0;

  // motion prediction

  motion_prediction_ = new MotionPredictionImu(nh_, nh_private_);

  // subscribers

  odom_subscriber_ = nh_.subscribe(
    "odom_fallback", 5, &MotionEstimation::odomCallback, this);
}

MotionEstimation::~MotionEstimation()
{

}

tf::Transform MotionEstimation::getMotionEstimation(RGBDFrame& frame)
{
  boost::mutex::scoped_lock(mutex_);

  // **** motion prediction // TODO: disable for now

  tf::Transform prediction;
  //motion_prediction_->getMotion(prediction);
  prediction.setIdentity();

  tf::Transform motion;
  bool result;

  if (frame.n_valid_keypoints < min_feature_count_)
  {
    ROS_WARN("Not enough features (%d detected, min is %d)", 
      frame.n_valid_keypoints, min_feature_count_);
    result = false;
  }
  else
  {
    // **** motion estimation
    result = getMotionEstimationImpl(frame, prediction, motion);
  }

  if (!result)
  {
    // **** fallback behaviors

    ROS_WARN("Could not estimate motion from RGBD data.");

    if (have_odom_msg_)
    {
      ROS_WARN("Using fallback odometry.");

      tf::Transform last_used_odom_tf, latest_odom_tf;

      tf::poseMsgToTF(last_used_odom_.pose.pose, last_used_odom_tf);
      tf::poseMsgToTF(latest_odom_.pose.pose, latest_odom_tf);

      motion = (last_used_odom_tf * b2c_).inverse() * (latest_odom_tf * b2c_);
      last_used_odom_ = latest_odom_;
    }
    else
    {
      ROS_WARN("Using Identity transform.");
      motion.setIdentity();
    }
  }
  
  // **** apply DoF constraints
  // this shoul dbe done inside impl classes
  //constrainMotion(motion);

  return motion;
}

void MotionEstimation::constrainMotion(tf::Transform& motion)
{
  // **** degree-of-freedom constraints

  if (motion_constraint_ == ROLL_PITCH)
  {
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, tf::getYaw(motion.getRotation()));

    motion.setRotation(q); 
  }
  else if (motion_constraint_ == ROLL_PITCH_Z)
  {
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, tf::getYaw(motion.getRotation()));
    
    tf::Vector3 p(motion.getOrigin().getX(), motion.getOrigin().getY(), 0.0);
    
    motion.setOrigin(p);
    motion.setRotation(q); 
  }

}

void MotionEstimation::setBaseToCameraTf(const tf::Transform& b2c)
{
  b2c_ = b2c;
}

void MotionEstimation::odomCallback(const OdomMsg::ConstPtr& odom_msg)
{
  boost::mutex::scoped_lock(mutex_);

  if (!have_odom_msg_)
  {
    have_odom_msg_ = true;
    last_used_odom_ = *odom_msg;
  }

  latest_odom_ = *odom_msg;  
}

} // namespace ccny_rgbd
