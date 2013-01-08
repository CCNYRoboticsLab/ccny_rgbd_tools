#include "ccny_rgbd/apps/keyframe_loop_mapper.h"

namespace ccny_rgbd
{

KeyframeLoopMapper::KeyframeLoopMapper(ros::NodeHandle nh, ros::NodeHandle nh_private):
  KeyframeMapper(nh, nh_private)
{
  ROS_INFO("Starting RGBD Keyframe Loop Mapper");

  associations_pub_ = nh_.advertise<visualization_msgs::Marker>( 
    "keyframe_associations", 1);

  generate_associations_service_ = nh_.advertiseService(
    "generate_associations", &KeyframeLoopMapper::generateAssociationsSrvCallback, this);
  add_manual_association_service_ = nh_.advertiseService(
    "add_manual_association", &KeyframeLoopMapper::addManualAssociationSrvCallback, this);
  solve_loop_service_ = nh_.advertiseService(
    "solve_loop", &KeyframeLoopMapper::solveLoopSrvCallback, this);

  loop_detector_ = new KeyframeLoopDetector(nh, nh_private);

  loop_solver_ = new KeyframeLoopSolverTORO(nh, nh_private);
}

KeyframeLoopMapper::~KeyframeLoopMapper()
{

}

void KeyframeLoopMapper::RGBDCallback(
  const ImageMsg::ConstPtr& depth_msg,
  const ImageMsg::ConstPtr& rgb_msg,
  const CameraInfoMsg::ConstPtr& info_msg)
{
  KeyframeMapper::RGBDCallback(depth_msg, rgb_msg, info_msg);

  //ROS_INFO("KeyframeLoopMapper::RGBDCallback");
}

bool KeyframeLoopMapper::generateAssociationsSrvCallback(
  GenerateAssociations::Request& request,
  GenerateAssociations::Response& response)
{
  associations_.clear();
  loop_detector_->generateKeyframeAssociations(keyframes_, associations_);

  publishKeyframeAssociations();

  return true;
}

bool KeyframeLoopMapper::addManualAssociationSrvCallback(
  AddManualAssociation::Request& request,
  AddManualAssociation::Response& response)
{
  int kf_idx_a = request.a;
  int kf_idx_b = request.b;

  // TODO: check for out of bounds

  bool result = loop_detector_->addManualAssociation(
    kf_idx_a, kf_idx_b, keyframes_, associations_);

  publishKeyframeAssociations();

  return result;
}

bool KeyframeLoopMapper::solveLoopSrvCallback(
  SolveLoop::Request& request,
  SolveLoop::Response& response)
{
  loop_solver_->solve(keyframes_, associations_);

  publishKeyframeAssociations();

  return true;
}

void KeyframeLoopMapper::publishKeyframeAssociations()
{
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = fixed_frame_;
  marker.ns = "RANSAC";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.points.resize(associations_.size() * 2);
  marker.scale.x = 0.001;

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  for (unsigned int as_idx = 0; as_idx < associations_.size(); ++as_idx)
  {
    // set up shortcut references
    const KeyframeAssociation& association = associations_[as_idx];
    int kf_idx_a = association.kf_idx_a;
    int kf_idx_b = association.kf_idx_b;
    RGBDKeyframe& keyframe_a = keyframes_[kf_idx_a];
    RGBDKeyframe& keyframe_b = keyframes_[kf_idx_b];

    int idx_start = as_idx*2;
    int idx_end   = as_idx*2 + 1;

    // start point for the edge
    marker.points[idx_start].x = keyframe_a.pose.getOrigin().getX();  
    marker.points[idx_start].y = keyframe_a.pose.getOrigin().getY();
    marker.points[idx_start].z = keyframe_a.pose.getOrigin().getZ();

    // end point for the edge
    marker.points[idx_end].x = keyframe_b.pose.getOrigin().getX();  
    marker.points[idx_end].y = keyframe_b.pose.getOrigin().getY();
    marker.points[idx_end].z = keyframe_b.pose.getOrigin().getZ();
  }

  associations_pub_.publish(marker);
}

} // namespace ccny_rgbd
