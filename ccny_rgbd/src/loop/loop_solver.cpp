#include "ccny_rgbd/loop/loop_solver.h"

namespace ccny_rgbd
{

LoopSolver::LoopSolver(ros::NodeHandle nh, ros::NodeHandle nh_private):
  KeyframeGenerator(nh, nh_private)
{
  solve_loop_service_ = nh_.advertiseService(
    "solve_loop", &LoopSolver::solveLoopSrvCallback, this);
  generate_associations_service_ = nh_.advertiseService(
    "generate_associations", &LoopSolver::generateAssociationsSrvCallback, this);

  associations_pub_ = nh_.advertise<visualization_msgs::Marker>( 
    "keyframe_associations", 1);
}

LoopSolver::~LoopSolver()
{

}

bool LoopSolver::solveLoopSrvCallback(
  ccny_rgbd::SolveLoop::Request& request,
  ccny_rgbd::SolveLoop::Response& response)
{
  ROS_INFO("Solve loop service called");
  solve();
  ROS_INFO("Solve loop service finished");
  return 0;
}

bool LoopSolver::generateAssociationsSrvCallback(
  ccny_rgbd::SolveLoop::Request& request,
  ccny_rgbd::SolveLoop::Response& response)
{
  ROS_INFO("Generate associations service called");
  generateAssociations();
  ROS_INFO("Generate associations service finished");
  return 0;
}

void LoopSolver::generateAssociations()
{
  // **** generate asssociations using RANSAC *****************

  LoopDetection loop_detection(nh_, nh_private_);
  loop_detection.generateKeyframeAssociations(keyframes_, associations_);
  publishKeyframeAssociations();
}


void LoopSolver::publishKeyframeAssociations()
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
