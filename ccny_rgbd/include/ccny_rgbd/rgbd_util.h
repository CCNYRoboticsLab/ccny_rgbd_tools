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

/** @brief Given a transform, calculates the linear and angular 
 * distance between it and identity
 * 
 * @param motion the incremental motion
 * @param dist reference to linear distance
 * @param angle reference to angular distance
 */
void getTfDifference(const tf::Transform& motion, double& dist, double& angle);

/** @brief Given two transformss, calculates the linear and angular 
 * distance between them, or the linear and angular components of
 * a.inv() * b
 *
 * @param a the first transform
 * @param b the second transform
 * @param dist reference to the linear distance
 * @param angle reference to the angular distance
 */
void getTfDifference(
  const tf::Transform& a, 
  const tf::Transform b, 
  double& dist, double& angle);

/** @brief Given a transfom (possibly computed as a difference between two transforms)
 * checks if either its angular or linar component exceeds a threshold
 * 
 * @param a input transform
 * @param dist linear distance trheshold
 * @param angle angular distance threshold
 */
bool tfGreaterThan(const tf::Transform& a, double dist, double angle);

/** @brief Converts an Eigen transform to a tf::Transform
 * @param trans Eigen transform
 * @return tf version of the eigen transform
 */
tf::Transform tfFromEigen(Eigen::Matrix4f trans);

/** @brief Converts an tf::Transform transform to an Eigen transform
 * @param tf the tf transform
 * @return the Eigen version of the transform
 */
Eigen::Matrix4f eigenFromTf(const tf::Transform& tf);

/** @brief Decomposes a tf into an Eigen 3x3 rotation matrix
 * and Eigen 3x1 rotation vector 
 *
 * @param tf the transform
 * @param R reference to the 3x3 Eigen matrix
 * @param t reference to the 3x1 Eigen vector
 */
void tfToEigenRt(
  const tf::Transform& tf, 
  Matrix3f& R, 
  Vector3f& t);

/** @brief Decomposes a tf::Transform into a 3x3 OpenCV rotation matrix
 * and a 3x1 OpenCV translation vector
 * 
 * @param transform the transform
 * @param R reference to the 3x3 OpenCV matrix
 * @param t reference to the 3x1 OpenCV vector
 */
void tfToOpenCVRt(
  const tf::Transform& transform,
  cv::Mat& R,
  cv::Mat& t);

/** @brief Creates a tf transform from a 3x3 OpenCV rotation matrix
 * and a 3x1 OpenCV translation vector
 * 
 * @param R the 3x3 OpenCV matrix
 * @param t the 3x1 OpenCV vector
 * @param transform reference to the output transform
 */
void openCVRtToTf(
  const cv::Mat& R,
  const cv::Mat& t,
  tf::Transform& transform);

/** @brief Decomposes a tf::Transform into x, y, z, roll, pitch, yaw
 * 
 * @param t the input transform
 * @param x the output x component
 * @param y the output y component
 * @param z the output z component
 * @param roll the output roll component
 * @param pitch the output pitch component
 * @param yaw the output yaw component
 */
void tfToXYZRPY(
  const tf::Transform& t,
  double& x,    double& y,     double& z,
  double& roll, double& pitch, double& yaw);

/** @brief Create OpenCV matrices from a CameraInfoMsg
 * 
 * @param camera_info_msg input CameraInfoMsg
 * @param intr output OpenCV intrinsic matrix
 * @param dist output OpenCV distortion vector
 */
void convertCameraInfoToMats(
  const CameraInfoMsg::ConstPtr camera_info_msg,
  cv::Mat& intr,
  cv::Mat& dist);

/** @brief Create CameraInfoMsg from OpenCV matrices (assumes no 
 * distortion)
 * 
 * @param intr input OpenCV intrinsic matrix
 * @param camera_info_msg output CameraInfoMsg
 */
void convertMatToCameraInfo(
  const cv::Mat& intr,
  CameraInfoMsg& camera_info_msg);

/** @brief Returns the duration, in ms, from a given time
 * 
 * @param start the start time
 * @return duration (in ms) from start until now
 */
double getMsDuration(const ros::WallTime& start);

/** @brief Filters out a vector of means given a mask of valid 
 * entries
 * 
 * @param means input vector of 3x1 matrices
 * @param valid vector mask of valid flags
 * @param means_f output vector of 3x1 matrices
 */
void removeInvalidMeans(
  const Vector3fVector& means,
  const BoolVector& valid,
  Vector3fVector& means_f);

/** @brief Filters out a vector of means and a vector of 
 * covariances given a mask of valid entries
 * 
 * @param means input vector of 3x1 matrices
 * @param covariances input vector of 3x3 matrices
 * @param valid vector mask of valid flags
 * @param means_f output vector of 3x1 matrices
 * @param covariances_f output vector of 3x1 matrices
 */
void removeInvalidDistributions(
  const Vector3fVector& means,
  const Matrix3fVector& covariances,
  const BoolVector& valid,
  Vector3fVector& means_f,
  Matrix3fVector& covariances_f);

/** @brief Transforms a vector of means
 * 
 * @param means vector of 3x1 matrices of positions (3D means)
 * @param transform the tranformation to be applied to all the means
 */
void transformMeans(
  Vector3fVector& means,
  const tf::Transform& transform);

/** @brief Transforms a vector of means and covariances
 * 
 * @param means vector of 3x1 matrices of positions (3D means)
 * @param covariances vector of 3x3 covariance matrices
 * @param transform the tranformation to be applied to all the means and covariances
 */
void transformDistributions(
  Vector3fVector& means,
  Matrix3fVector& covariances,
  const tf::Transform& transform);

/** @brief Creates a pcl point cloud form a vector
 * of eigen matrix means
 * 
 * @param means vector of 3x1 matrices of positions (3D means)
 * @param cloud reference to the output cloud
 */
void pointCloudFromMeans(
  const Vector3fVector& means,
  PointCloudFeature& cloud);

/** @brief reprojects a depth image to another depth image,
 * registered in the rgb camera's frame. 
 * 
 * Both images need to be rectified first. ir2rgb is a matrix 
 * such that for any point P_IR in the depth camera frame
 * P_RGB = ir2rgb * P_IR
 *
 * @param intr_rect_ir intrinsic matrix of the rectified depth image
 * @param intr_rect_rgb intrinsic matrix of the rectified RGB image
 * @param ir2rgb extrinsic matrix between the IR(depth) and RGB cameras
 * @param depth_img_rect the input image: rectified depth image
 * @param depth_img_rect_reg the output image: rectified and registered into the 
 *        RGB frame
 */
void buildRegisteredDepthImage(
  const cv::Mat& intr_rect_ir,
  const cv::Mat& intr_rect_rgb,
  const cv::Mat& ir2rgb,
  const cv::Mat& depth_img_rect,
  cv::Mat& depth_img_rect_reg);

/** @brief Constructs a point cloud, a depth image and instrinsic matrix
 * 
 * @param depth_img_rect rectified depth image (16UC1, in mm) 
 * @param intr_rect_ir intinsic matrix of the rectified depth image
 * @param cloud reference to teh output point cloud
 */
void buildPointCloud(
  const cv::Mat& depth_img_rect,
  const cv::Mat& intr_rect_ir,
  PointCloudT& cloud);

/** @brief Constructs a point cloud with color
 * 
 * Prior to calling this functions, both images need to be rectified, 
 * and the depth image has to be registered into the frame of the RGB image.
 * 
 * @param depth_img_rect_reg rectified and registered depth image (16UC1, in mm) 
 * @param rgb_img_rect rectified rgb image (8UC3)
 * @param intr_rect_rgb intrinsic matrix
 * @param cloud reference to the output point cloud
 */
void buildPointCloud(
  const cv::Mat& depth_img_rect_reg,
  const cv::Mat& rgb_img_rect,
  const cv::Mat& intr_rect_rgb,
  PointCloudT& cloud);

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_UTIL_H
