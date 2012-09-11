#include "ccny_rgbd/mapping/keyframe_generator.h"

namespace ccny_rgbd
{

KeyframeGenerator::KeyframeGenerator(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  // *** init params

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("kf/kf_dist_eps", kf_dist_eps_))
    kf_dist_eps_  = 0.10;
  if (!nh_private_.getParam ("kf/kf_angle_eps", kf_angle_eps_))
    kf_angle_eps_  = 10.0 * M_PI / 180.0;
}

KeyframeGenerator::~KeyframeGenerator()
{

}

bool KeyframeGenerator::processFrame(
  const RGBDFrame& frame, 
  const tf::Transform& pose)
{
  bool result; // if true, add new frame

  if(keyframes_.empty())
  {
    result = true;
  }
  else
  {
    double dist, angle;
    getTfDifference(pose, keyframes_.back().pose, dist, angle);

    if (dist > kf_dist_eps_ || angle > kf_angle_eps_)
      result = true;
    else 
      result = false;
  }

  if (result) addKeyframe(frame, pose);
  return result;
}

void KeyframeGenerator::addKeyframe(
  const RGBDFrame& frame, 
  const tf::Transform& pose)
{
  ROS_INFO("Adding frame");
  RGBDKeyframe keyframe(frame);
  keyframe.pose = pose;
  //keyframe.constructDataCloud();
  keyframes_.push_back(keyframe);
}

} // namespace ccny_rgbd
