#ifndef CCNY_RGBD_LOOP_SOLVER_GICP_H
#define CCNY_RGBD_LOOP_SOLVER_GICP_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <ccny_gicp/gicp.h>
#include <ccny_gicp/types.h>
#include <ccny_gicp/gicp_align.h>

#include "ccny_rgbd/loop/loop_solver.h"
#include "ccny_rgbd_vo/AlignPair.h"

namespace ccny_rgbd
{

typedef Eigen::aligned_allocator<PointCloudT> PointCloudTAllocator;
typedef std::vector<PointCloudT, PointCloudTAllocator> PointCloudTVector;

class LoopSolverGICP: public LoopSolver
{
  public:

    LoopSolverGICP(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~LoopSolverGICP();
 
    void solve();

    bool alignPairSrvCallback(
      ccny_rgbd_vo::AlignPair::Request&  request,
      ccny_rgbd_vo::AlignPair::Response& response);

  private:

    double vgf_res_;
    int nn_count_;
    double gicp_epsilon_;
    double vgf_range_;
    int window_size_;

    ccny_gicp::GICPAlign reg_;

    ros::ServiceServer align_pair_service_;

    void alignPair(int idx_a, int idx_b);
};

} //namespace ccny_rgbd

#endif //  CCNY_RGBD_LOOP_SOLVER_GICP_H
