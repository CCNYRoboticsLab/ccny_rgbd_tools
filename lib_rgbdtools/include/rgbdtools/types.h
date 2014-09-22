/**
 *  @file types.h
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

#ifndef RGBDTOOLS_TYPES_H
#define RGBDTOOLS_TYPES_H

#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transformation_estimation_svd.h>

namespace rgbdtools {

// Eigen matrix types

typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Affine3f Pose;
typedef Eigen::Affine3f AffineTransform;
typedef Eigen::Matrix<double,6,6> InformationMatrix;

// Vector types

typedef std::vector<int>             IntVector;
typedef std::vector<float>           FloatVector;
typedef std::vector<bool>            BoolVector;
typedef std::vector<cv::Point2f>     Point2fVector;
typedef std::vector<cv::Point3f>     Point3fVector;
typedef std::vector<Eigen::Matrix3f> Matrix3fVector;
typedef std::vector<Eigen::Vector3f> Vector3fVector;
typedef std::vector<cv::KeyPoint>    KeypointVector;
typedef std::vector<cv::DMatch>      DMatchVector;

typedef Eigen::aligned_allocator<AffineTransform> AffineTransformAllocator; 
typedef std::vector<AffineTransform, AffineTransformAllocator> AffineTransformVector;

// PCL types

typedef pcl::PointXYZRGB              PointT;
typedef pcl::PointCloud<PointT>       PointCloudT;

typedef pcl::PointXYZ                 PointFeature;
typedef pcl::PointCloud<PointFeature> PointCloudFeature;

typedef pcl::KdTreeFLANN<PointFeature> KdTree;
typedef pcl::registration::TransformationEstimationSVD<PointFeature, PointFeature> TransformationEstimationSVD; 

} // namespace rgbdtools

#endif // RGBDTOOLS_TYPES_H
