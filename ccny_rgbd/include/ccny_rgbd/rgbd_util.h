/*
 *  Copyright (C) 2013, City University of New York
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  CCNY Robotics Lab
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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

/* reprojects a depth image to another depth image,
 * registered in the rgb camera's frame. Both images 
 * need to be rectified first. ir2rgb is a matrix such that 
 * for any point P_IR in the depth camera frame
 * P_RGB = ir2rgb * P
 */
void buildRegisteredDepthImage(
  const cv::Mat& intr_rect_ir,
  const cv::Mat& intr_rect_rgb,
  const cv::Mat& ir2rgb,
  const cv::Mat& depth_img_rect,
  cv::Mat& depth_img_rect_reg);

/* Constructs a point cloud, given:
 *  - a depth image (uint16_t, mm) which has been undistorted
 *  - the intinsic matrix of the depth image after rectification
 */
void buildPointCloud(
  const cv::Mat& depth_img_rect,
  const cv::Mat& intr_rect_ir,
  PointCloudT& cloud);

/* Constructs a point cloud with color, given: 
 *  - a depth image (uint16_t, mm) which has been undistorted 
 *    and registeredin the rgb frame, 
 *  - an an rgb image which has been undistorted
 *  - the intinsic matrix of the RGB image after rectification
 */
void buildPointCloud(
  const cv::Mat& depth_img_rect_reg,
  const cv::Mat& rgb_img_rect,
  const cv::Mat& intr_rect_rgb,
  PointCloudT& cloud);

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_UTIL_H
