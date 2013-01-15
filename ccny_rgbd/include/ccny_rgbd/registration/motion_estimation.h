#ifndef CCNY_RGBD_MOTION_ESTIMATION_H
#define CCNY_RGBD_MOTION_ESTIMATION_H

#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/mutex.hpp>

#include "ccny_rgbd/structures/rgbd_frame.h"

namespace ccny_rgbd
{

class MotionEstimation
{
  typedef nav_msgs::Odometry OdomMsg;

  enum MotionConstraint {NONE = 0, ROLL_PITCH = 1, ROLL_PITCH_Z = 2};

  public:

    MotionEstimation(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MotionEstimation();

    tf::Transform getMotionEstimation(RGBDFrame& frame);

    void setBaseToCameraTf(const tf::Transform& b2c);

    virtual int getModelSize() const { return 0; }

  protected:
  
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    tf::Transform b2c_; // Base (moving) frame to Camera-optical frame

    int min_feature_count_;
    int motion_constraint_;

    virtual bool getMotionEstimationImpl(
      RGBDFrame& frame,
      const tf::Transform& prediction,
      tf::Transform& motion) = 0;

    void constrainMotion(tf::Transform& motion);

  private:

};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MOTION_ESTIMATION_H
