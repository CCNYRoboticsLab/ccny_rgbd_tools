#ifndef CCNY_RGBD_RGBD_UTIL_H
#define CCNY_RGBD_RGBD_UTIL_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf/transform_datatypes.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>

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

// **** typedefs *********************************************

typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Vector3f Vector3f;

typedef std::vector<int>             IntVector;
typedef std::vector<float>           FloatVector;
typedef std::vector<bool>            BoolVector;
typedef std::vector<cv::Point2f>     Point2fVector;
typedef std::vector<cv::Point3f>     Point3fVector;
typedef std::vector<cv::Mat>         MatVector;
typedef std::vector<Eigen::Matrix3f> Matrix3fVector;
typedef std::vector<Eigen::Vector3f> Vector3fVector;
typedef std::vector<cv::KeyPoint>    KeypointVector;

typedef pcl::PointXYZRGB          PointT;
typedef pcl::PointCloud<PointT>   PointCloudT;

typedef pcl::PointXYZ PointFeature;
typedef pcl::PointCloud<PointFeature>   PointCloudFeature;


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

/* returns the duration, in ms, from a given time
 */
double getMsDuration(const ros::WallTime& start);

/* decomposes a tf::Transform into 6 x, t, z, d, p, y
 */
void getXYZRPY(const tf::Transform& t,
                     double& x,    double& y,     double& z,
                     double& roll, double& pitch, double& yaw);

/* decomposes a tf::Transform into a 3x3 OpenCV rotation matrix
 * and a 3x1 OpenCV translation vector
 */
void tfToCV(
  const tf::Transform& transform,
  cv::Mat& translation,
  cv::Mat& rotation);

/* get a 4x3 matrix from OpenCV r vector and t vector
 */
cv::Mat matrixFromRvecTvec(const cv::Mat& rvec, const cv::Mat& tvec);

/* get a 4x3 matrix from OpenCV 3x3 R matrix and t vector
 */
cv::Mat matrixFromRT(const cv::Mat& rmat, const cv::Mat& tvec);

/* get a 4x4 matrix from a 3x3 matrix, 
 * 4th row is 0 0 0 1
 */
cv::Mat m4(const cv::Mat& m3);

void transformDistributions(
  MatVector& means,
  MatVector& covariances,
  const tf::Transform& transform);

void getPointCloudFromDistributions(
  const MatVector& means,
  PointCloudFeature& cloud);

// **** EIGEN **********************************************

void removeInvalidFeatures(
  const Vector3fVector& means,
  const Matrix3fVector& covariances,
  const BoolVector& valid,
  Vector3fVector& means_f,
  Matrix3fVector& covariances_f);

void cvMatToEigenMatrix3f(
  const cv::Mat mat_cv, 
  Matrix3f& mat_eigen);

void cvMatToEigenVector3f(
  const cv::Mat mat_cv, 
  Vector3f& mat_eigen);

void tfToEigenRt(
  const tf::Transform& tf, 
  Matrix3f& R, 
  Vector3f& t);

void transformDistributions(
  Vector3fVector& means,
  Matrix3fVector& covariances,
  const tf::Transform& transform);

void getPointCloudFromDistributions(
  const Vector3fVector& means,
  PointCloudFeature& cloud);

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_UTIL_H
