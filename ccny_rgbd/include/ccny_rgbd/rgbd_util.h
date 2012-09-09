#ifndef CCNY_RGBD_RGBD_UTIL_H
#define CCNY_RGBD_RGBD_UTIL_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf/transform_datatypes.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/features2d/features2d.hpp>

// for campatibility b/n ROS Electric and Fuerte
#if ROS_VERSION_MINIMUM(1, 8, 0)
  typedef tf::Matrix3x3 MyMatrix;
#else
  typedef btMatrix3x3 MyMatrix;
#endif

namespace pcl
{

struct PointXYZ_Feature
{
  PCL_ADD_POINT4D;  // This adds the members x,y,z which can also be accessed using the point (which is float[4])

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

} // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZ_Feature,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z));

namespace ccny_rgbd
{

typedef pcl::PointXYZRGB          PointT;
typedef pcl::PointCloud<PointT>   PointCloudT;

typedef pcl::PointXYZ PointFeature;
typedef pcl::PointCloud<PointFeature>   PointCloudFeature;

void getTfDifference(const tf::Transform& a, const tf::Transform b, double& dist, double& angle);

bool tfGreaterThan(const tf::Transform& a, double dist, double angle);

tf::Transform tfFromEigen(Eigen::Matrix4f trans);

Eigen::Matrix4f eigenFromTf(const tf::Transform& tf);

void getXYZRPY(const tf::Transform& t,
                     double& x,    double& y,     double& z,
                     double& roll, double& pitch, double& yaw);

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_UTIL_H
