#include "ccny_rgbd/registration/motion_prediction_imu.h"

namespace ccny_rgbd
{

MotionPredictionImu::MotionPredictionImu(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  have_message_(false)
{
  // **** init variables

  // *** init params

  //if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
  //  fixed_frame_ = "/odom";

  imu_subscriber_ = nh_.subscribe(
    "imu/data", 5, &MotionPredictionImu::imuCallback, this);
}

MotionPredictionImu::~MotionPredictionImu()
{

}

void MotionPredictionImu::imuCallback(const ImuMsg::ConstPtr& imu_msg)
{
  boost::mutex::scoped_lock(mutex_);

  latest_imu_msg_ = *imu_msg;

  if (!have_message_)
  {
    last_used_imu_msg_ = latest_imu_msg_;   
    have_message_ = true;
  }
}

void MotionPredictionImu::getMotion(tf::Transform& tf)
{
  boost::mutex::scoped_lock(mutex_);

  if(!have_message_)
  { 
    tf.setIdentity();
  }
  else
  {
    tf::Quaternion q_new, q_old;
    tf::quaternionMsgToTF(latest_imu_msg_.orientation,    q_new);
    tf::quaternionMsgToTF(last_used_imu_msg_.orientation, q_old);

    tf::Transform t_new, t_old;
    t_new.setIdentity();
    t_old.setIdentity();
    t_new.setRotation(q_new);
    t_old.setRotation(q_old);

    tf = t_new * t_old.inverse();

    last_used_imu_msg_ = latest_imu_msg_;
  }
}

} // namespace ccny_rgbd
