#ifndef CCNY_RGBD_LOOP_KEYFRAME_SOLVER_TORO_H
#define CCNY_RGBD_LOOP_KEYFRAME_SOLVER_TORO_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <ccny_toro/treeoptimizer3.hh>
#include <ccny_toro/posegraph3.hh>
#include <ccny_toro/posegraph.hh>
#include <ccny_toro/transformation3.hh>

#include "ccny_rgbd/mapping/keyframe_loop_solver.h"

namespace ccny_rgbd
{

class KeyframeLoopSolverTORO: public KeyframeLoopSolver
{
  public:

    KeyframeLoopSolverTORO(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeLoopSolverTORO();
 
    void solve(KeyframeVector& keyframes,
               KeyframeAssociationVector& associations);

  private:


};

} //namespace ccny_rgbd

#endif //  CCNY_RGBD_LOOP_KEYFRAME_SOLVER_TORO_H
