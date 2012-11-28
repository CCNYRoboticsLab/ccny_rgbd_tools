#include "ccny_rgbd/mapping/keyframe_loop_solver.h"

namespace ccny_rgbd
{

KeyframeLoopSolver::KeyframeLoopSolver(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{

}

KeyframeLoopSolver::~KeyframeLoopSolver()
{

}

} // namespace ccny_rgbd
