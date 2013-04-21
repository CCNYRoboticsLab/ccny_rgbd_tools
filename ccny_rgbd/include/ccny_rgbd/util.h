/**
 *  @file rgbd_util.h
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2013, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
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
#include <cv_bridge/cv_bridge.h>

#include <rgbdtools/rgbdtools.h>

#include "ccny_rgbd/types.h"

namespace ccny_rgbd {

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

tf::Transform tfFromEigenAffine(const AffineTransform& trans);
AffineTransform eigenAffineFromTf(const tf::Transform& tf);

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

void createRGBDFrameFromROSMessages(
  const ImageMsg::ConstPtr& rgb_msg,
  const ImageMsg::ConstPtr& depth_msg,
  const CameraInfoMsg::ConstPtr& info_msg,
  rgbdtools::RGBDFrame& frame);

/** @brief Copies over the poses from a Eigen vector to a ROS message.
 * Assumes the message is already correctly resized, and preserves
 * the headers of each pose in the message
 */
void pathEigenAffineToROS(
  const AffineTransformVector& path,
  PathMsg& path_msg);

/** @brief copies over the poses from a ROS message Eigen vector.
 * The eigen vector will be cleared and resized appropriately.
 */
void pathROSToEigenAffine(
  const PathMsg& path_msg,
  AffineTransformVector& path);

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_UTIL_H
