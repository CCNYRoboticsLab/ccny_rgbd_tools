#include "ccny_rgbd/mapping/keyframe_generator.h"

namespace ccny_rgbd
{

KeyframeGenerator::KeyframeGenerator(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  manual_add_(false)
{
  // *** init params

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("kf/kf_dist_eps", kf_dist_eps_))
    kf_dist_eps_  = 0.10;
  if (!nh_private_.getParam ("kf/kf_angle_eps", kf_angle_eps_))
    kf_angle_eps_  = 10.0 * M_PI / 180.0;

  // **** services

  add_manual_keyframe_service_ = nh_.advertiseService(
    "add_manual_keyframe", &KeyframeGenerator::addManualKeyframeSrvCallback, this);
}

KeyframeGenerator::~KeyframeGenerator()
{

}

bool KeyframeGenerator::addManualKeyframeSrvCallback(
  AddManualKeyframe::Request& request,
  AddManualKeyframe::Response& response)
{
  manual_add_ = true;

  return true;
}

bool KeyframeGenerator::processFrame(
  const RGBDFrame& frame, 
  const tf::Transform& pose)
{
  bool result; // if true, add new frame

  if(keyframes_.empty() || manual_add_)
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
  //ROS_INFO("Adding frame");
  RGBDKeyframe keyframe(frame);
  keyframe.pose = pose;
  keyframe.constructDensePointCloud();

  if (manual_add_)
  {
    ROS_INFO("Adding frame manually");
    manual_add_ = false;
    keyframe.manually_added = true;
  }
  keyframes_.push_back(keyframe);
}


} // namespace ccny_rgbd
