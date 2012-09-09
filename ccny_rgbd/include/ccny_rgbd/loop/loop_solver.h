#ifndef CCNY_RGBD_LOOP_SOLVER_H
#define CCNY_RGBD_LOOP_SOLVER_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <sba/sba.h>
#include <sba/visualization.h>
#include <visualization_msgs/Marker.h>

#include "ccny_rgbd/loop/keyframe_generator.h"
#include "ccny_rgbd/loop/loop_detection.h"
#include "ccny_rgbd_vo/SolveLoop.h"
#include "ccny_rgbd_vo/GenerateAssociations.h"

namespace ccny_rgbd
{

class LoopSolver: public KeyframeGenerator
{
  public:

    LoopSolver(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~LoopSolver();
 
    bool solveLoopSrvCallback(
      ccny_rgbd_vo::SolveLoop::Request& request,
      ccny_rgbd_vo::SolveLoop::Response& response);

    bool generateAssociationsSrvCallback(
      ccny_rgbd_vo::SolveLoop::Request& request,
      ccny_rgbd_vo::SolveLoop::Response& response);

  protected:

    KeyframeAssociationVector associations_;

    virtual void solve() = 0;
    
    void generateAssociations();
    void publishKeyframeAssociations();

  private:

    ros::ServiceServer solve_loop_service_;
    ros::ServiceServer generate_associations_service_;

    ros::Publisher associations_pub_;
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_LOOP_SOLVER_H
