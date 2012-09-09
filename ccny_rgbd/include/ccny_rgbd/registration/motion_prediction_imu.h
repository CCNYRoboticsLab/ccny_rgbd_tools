#ifndef CCNY_RGBD_MOTION_PREDICTION_IMU_H
#define CCNY_RGBD_MOTION_ESTIMATION_IMU_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <boost/thread/mutex.hpp>

namespace ccny_rgbd
{

class MotionPredictionImu
{
  typedef sensor_msgs::Imu ImuMsg;

  public:

    MotionPredictionImu(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MotionPredictionImu();
 
    void getMotion(tf::Transform& tf);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber imu_subscriber_;

    // params

    std::string fixed_frame_; 
    std::string base_frame_;

    // variables
  
    boost::mutex mutex_;
    bool have_message_;
    ImuMsg latest_imu_msg_;
    ImuMsg last_used_imu_msg_;

    // functions

    void imuCallback(const ImuMsg::ConstPtr& imu_msg);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MOTION_ESTIMATION_IMU_H
