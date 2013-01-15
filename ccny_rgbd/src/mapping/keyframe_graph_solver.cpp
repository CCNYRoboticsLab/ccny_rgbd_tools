#include "ccny_rgbd/mapping/keyframe_graph_solver.h"

namespace ccny_rgbd
{

KeyframeGraphSolver::KeyframeGraphSolver(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{

}

KeyframeGraphSolver::~KeyframeGraphSolver()
{

}

} // namespace ccny_rgbd
