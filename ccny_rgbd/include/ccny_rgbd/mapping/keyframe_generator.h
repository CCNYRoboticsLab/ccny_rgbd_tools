#ifndef CCNY_RGBD_KEYFRAME_GENERATOR_H
#define CCNY_RGBD_KEYFRAME_GENERATOR_H

#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/structures/rgbd_frame.h"
#include "ccny_rgbd/structures/rgbd_keyframe.h"

namespace ccny_rgbd
{

class KeyframeGenerator
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyframeGenerator(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeGenerator();

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    KeyframeVector keyframes_;
    std::string fixed_frame_;

    ros::ServiceServer pairwise_ransac_service_;

    bool processFrame(const RGBDFrame& frame, const tf::Transform& pose);

  private:
  
    double kf_dist_eps_;
    double kf_angle_eps_;

    void addKeyframe(const RGBDFrame& frame, const tf::Transform& pose);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_GENERATOR_H
