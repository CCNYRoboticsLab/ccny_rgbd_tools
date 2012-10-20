#ifndef CCNY_RGBD_LOOP_KEYFRAME_SOLVER_SBA_H
#define CCNY_RGBD_LOOP_KEYFRAME_SOLVER_SBA_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sba/sba.h>
#include <sba/visualization.h>

#include "ccny_rgbd/mapping/keyframe_loop_solver.h"

namespace ccny_rgbd
{

class KeyframeLoopSolverSBA: public KeyframeLoopSolver
{
  public:

    KeyframeLoopSolverSBA(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeLoopSolverSBA();
 
    void solve(KeyframeVector& keyframes,
               KeyframeAssociationVector& associations);

  private:

    void setUp(KeyframeVector& keyframes,
               KeyframeAssociationVector& associations);

    sba::SysSBA sys_sba_;

    ros::Publisher cam_marker_pub_;
    ros::Publisher point_marker_pub_;
    ros::Publisher cam_marker_s_pub_;
    ros::Publisher point_marker_s_pub_;  
};

} //namespace ccny_rgbd

#endif //  CCNY_RGBD_LOOP_KEYFRAME_SOLVER_SBA_H
