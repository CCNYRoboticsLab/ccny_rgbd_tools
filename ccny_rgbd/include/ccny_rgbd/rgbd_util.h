#ifndef CCNY_RGBD_RGBD_UTIL_H
#define CCNY_RGBD_RGBD_UTIL_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf/transform_datatypes.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>

namespace ccny_rgbd
{

// **** typedefs *********************************************

typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Vector3f Vector3f;

typedef std::vector<int>             IntVector;
typedef std::vector<float>           FloatVector;
typedef std::vector<bool>            BoolVector;
typedef std::vector<cv::Point2f>     Point2fVector;
typedef std::vector<cv::Point3f>     Point3fVector;
typedef std::vector<Eigen::Matrix3f> Matrix3fVector;
typedef std::vector<Eigen::Vector3f> Vector3fVector;
typedef std::vector<cv::KeyPoint>    KeypointVector;

typedef pcl::PointXYZRGB          PointT;
typedef pcl::PointCloud<PointT>   PointCloudT;

typedef pcl::PointXYZ PointFeature;
typedef pcl::PointCloud<PointFeature>   PointCloudFeature;

// **** util functions **************************************

/* given a transform, calculates the linear and angular 
 * distance between it and identity
 */
void getTfDifference(const tf::Transform& motion, double& dist, double& angle);

/* given two tf::transforms, calculates the linear and angular 
 * distance between them
 */
void getTfDifference(const tf::Transform& a, const tf::Transform b, double& dist, double& angle);

/* given a tf::Transfom (possibly computed as a difference between two transforms)
 * checks if either its angular or linar component exedds a threshold
 */
bool tfGreaterThan(const tf::Transform& a, double dist, double angle);

/* converts and Eigen transform to a tf::Transform
 */
tf::Transform tfFromEigen(Eigen::Matrix4f trans);

/* converts and tf::Transform transform to an Eigen transform
 */
Eigen::Matrix4f eigenFromTf(const tf::Transform& tf);

/* decomposes a tf into an Eigen 3x3 rotation matrix
 * and Eigen 3x1 rotation vector */
void tfToEigenRt(
  const tf::Transform& tf, 
  Matrix3f& R, 
  Vector3f& t);

/* decomposes a tf::Transform into 6 x, t, z, d, p, y
 */
void getXYZRPY(const tf::Transform& t,
                     double& x,    double& y,     double& z,
                     double& roll, double& pitch, double& yaw);

/* returns the duration, in ms, from a given time
 */
double getMsDuration(const ros::WallTime& start);

void removeInvalidMeans(
  const Vector3fVector& means,
  const BoolVector& valid,
  Vector3fVector& means_f);

void removeInvalidDistributions(
  const Vector3fVector& means,
  const Matrix3fVector& covariances,
  const BoolVector& valid,
  Vector3fVector& means_f,
  Matrix3fVector& covariances_f);

void transformMeans(
  Vector3fVector& means,
  const tf::Transform& transform);

void transformDistributions(
  Vector3fVector& means,
  Matrix3fVector& covariances,
  const tf::Transform& transform);

void pointCloudFromMeans(
  const Vector3fVector& means,
  PointCloudFeature& cloud);

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_UTIL_H
