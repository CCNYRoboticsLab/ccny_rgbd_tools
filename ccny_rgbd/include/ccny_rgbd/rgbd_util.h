#ifndef CCNY_RGBD_RGBD_UTIL_H
#define CCNY_RGBD_RGBD_UTIL_H

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tf/transform_datatypes.h>
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>

#include "ccny_rgbd/types.h"

namespace ccny_rgbd
{

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

/* decomposes a tf::Transform into a 3x3 OpenCV rotation matrix
 * and a 3x1 OpenCV translation vector
 */
void tfToOpenCVRt(
  const tf::Transform& transform,
  cv::Mat& R,
  cv::Mat& t);

// TODO: description
void openCVRtToTf(
  const cv::Mat& R,
  const cv::Mat& t,
  tf::Transform& transform);

/* decomposes a tf::Transform into x, y, z, roll, pitch, yaw
 * TODO: rename to tfToXYZRPY
 */
void getXYZRPY(
  const tf::Transform& t,
  double& x,    double& y,     double& z,
  double& roll, double& pitch, double& yaw);

// TODO: comment
void convertCameraInfoToMats(
  const CameraInfoMsg::ConstPtr camera_info_msg,
  cv::Mat& intr,
  cv::Mat& dist);

// TODO: comment
void convertMatToCameraInfo(
  const cv::Mat& intr,
  CameraInfoMsg& camera_info_msg);

/* returns the duration, in ms, from a given time
 */
double getMsDuration(const ros::WallTime& start);

/* filters out a vector of means given a mask of valid 
 * entries
 */
void removeInvalidMeans(
  const Vector3fVector& means,
  const BoolVector& valid,
  Vector3fVector& means_f);

/* filters out a vector of means and a vector of 
 * covariances given a mask of valid entries
 */
void removeInvalidDistributions(
  const Vector3fVector& means,
  const Matrix3fVector& covariances,
  const BoolVector& valid,
  Vector3fVector& means_f,
  Matrix3fVector& covariances_f);

/* transforms a vector of means
 */
void transformMeans(
  Vector3fVector& means,
  const tf::Transform& transform);

/* transforms a vector of means and covariances
 */
void transformDistributions(
  Vector3fVector& means,
  Matrix3fVector& covariances,
  const tf::Transform& transform);

/* creates a pcl point cloud form a vector
 * of eigen matrix means
 */
void pointCloudFromMeans(
  const Vector3fVector& means,
  PointCloudFeature& cloud);

/* generates an RGB and depth images from the projection of a point cloud
 */
void projectCloudToImage(const PointCloudT& cloud,
                         const Matrix3f& rmat,
                         const Vector3f& tvec,
                         const Matrix3f& intrinsic,
                         uint width,
                         uint height,
                         cv::Mat& rgb_img,
                         cv::Mat& depth_img);

/* Finds the PnP transformation of the reference image based on the virtual images
 */
void tfFromImagePair(
  const cv::Mat& reference_img,
  const cv::Mat& virtual_img,
  const cv::Mat& virtual_depth_img,
  const Matrix3f& intrinsic_matrix,
  tf::Transform& transform);

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_UTIL_H
