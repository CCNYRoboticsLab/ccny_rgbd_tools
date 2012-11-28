#ifndef CCNY_GICP_TYPES_H
#define CCNY_GICP_TYPES_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include "ccny_gicp/octree_pointcloud_storage.h"
#include "ccny_gicp/octree_pointcloud_storage.hpp"

typedef double gicp_mat_t[3][3];

struct PointGICP 
{
  PCL_ADD_POINT4D;
  float range;
  float rgb;
  gicp_mat_t C; // covariance matrix
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;;
    
POINT_CLOUD_REGISTER_POINT_STRUCT (PointGICP,           // here we assume a XYZ + "test" (as fields)
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, range, range)
                                    (float, rgb, rgb)
                                    (gicp_mat_t, C, C)
)

struct GICPParams
{
  double max_distance;
  bool solve_rotation;
  bool debug;
  int max_iteration;
  int max_iteration_inner;
  double epsilon;
  double epsilon_rot;
};

typedef pcl::PointCloud<PointGICP>  PointCloudGICP;

typedef std::vector<int>   IntVector;
typedef std::vector<float> FloatVector;

typedef pcl::KdTreeFLANN<PointGICP> KdTree;
typedef pcl::octree::OctreePointCloudStorage<PointGICP> Octree;

#endif // CCNY_GICP_TYPES_H
