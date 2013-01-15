#ifndef CCNY_RGBD_KEYFRAME_GRAPH_SOLVER_H
#define CCNY_RGBD_KEYFRAME_GRAPH_SOLVER_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>

#include "ccny_rgbd/structures/rgbd_keyframe.h"
#include "ccny_rgbd/structures/keyframe_association.h"

namespace ccny_rgbd
{

class KeyframeGraphSolver
{
  public:

    KeyframeGraphSolver(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeGraphSolver();
 
    virtual void solve(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations) = 0;

  protected:
 
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
   
  private:

};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_GRAPH_SOLVER_H
