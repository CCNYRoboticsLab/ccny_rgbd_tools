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
#include <pcl/filters/voxel_grid.h>
#include <opencv2/opencv.hpp>

#include "ccny_rgbd/types.h"

namespace ccny_rgbd {

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

/** @brief Constructs a point cloud, a depth image and intrinsic matrix
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

/** @brief converts a 32FC1 depth image (in meters) to a
 * 16UC1 depth image (in mm).
 *
 * @param depth_image_in the input 32FC1 image
 * @param depth_image_out the output 16UC1 image
 */
void depthImageFloatTo16bit(
  const cv::Mat& depth_image_in,
  cv::Mat& depth_image_out);

void eigenAffineToOpenCVRt(
  const AffineTransform& transform,
  cv::Mat& R,
  cv::Mat& t);
  
void openCVRtToEigenAffine(
  const cv::Mat& R,
  const cv::Mat& t,
  AffineTransform& eigen_affine);

void eigenAffineToXYZRPY(
  const AffineTransform& transform, 
  float& x, float& y, float& z, 
  float& roll, float& pitch, float& yaw);
 
void XYZRPYToEigenAffine(
  float x, float y, float z, 
  float roll, float pitch, float yaw, 
  AffineTransform& ttransform);

void transformDistributions(
  Vector3fVector& means,
  Matrix3fVector& covariances,
  const AffineTransform& transform);

void getTfDifference(
  const AffineTransform& transform, 
  double& dist, double& angle);

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_UTIL_H
