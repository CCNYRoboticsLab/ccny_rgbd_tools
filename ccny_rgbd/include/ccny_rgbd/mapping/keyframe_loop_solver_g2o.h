#ifndef CCNY_RGBD_KEYFRAME_LOOP_SOLVER_G2O_H
#define CCNY_RGBD_KEYFRAME_LOOP_SOLVER_G2O_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <g2o/core/graph_optimizer_sparse.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam3d/types_six_dof_quat.h>

#include "ccny_rgbd/mapping/keyframe_loop_solver.h"

namespace ccny_rgbd
{

class KeyframeLoopSolverG2O: public KeyframeLoopSolver
{
  public:

    KeyframeLoopSolverG2O(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeLoopSolverG2O();
 
    void solve(KeyframeVector& keyframes,
               KeyframeAssociationVector& associations);

  private:


};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_LOOP_SOLVER_G2O_H
