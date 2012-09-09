#ifndef CCNY_RGBD_LOOP_SOLVER_SBA_H
#define CCNY_RGBD_LOOP_SOLVER_SBA_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sba/sba.h>
#include <sba/visualization.h>

#include "ccny_rgbd/loop/loop_solver.h"

namespace ccny_rgbd
{

class LoopSolverSBA: public LoopSolver
{
  public:

    LoopSolverSBA(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~LoopSolverSBA();
 
    void solve();

  private:

    void setUp();

    sba::SysSBA sys_sba_;

    ros::Publisher cam_marker_pub_;
    ros::Publisher point_marker_pub_;
    ros::Publisher cam_marker_s_pub_;
    ros::Publisher point_marker_s_pub_;  
};

} //namespace ccny_rgbd

#endif //  CCNY_RGBD_LOOP_SOLVER_SBA_H
