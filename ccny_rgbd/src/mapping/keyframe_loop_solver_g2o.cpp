#include "ccny_rgbd/mapping/keyframe_loop_solver_g2o.h"

namespace ccny_rgbd
{

KeyframeLoopSolverG2O::KeyframeLoopSolverG2O(ros::NodeHandle nh, ros::NodeHandle nh_private):
  KeyframeLoopSolver(nh, nh_private)
{

}

KeyframeLoopSolverG2O::~KeyframeLoopSolverG2O()
{

}

void KeyframeLoopSolverG2O::solve(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
 

}

} // namespace ccny_rgbd
