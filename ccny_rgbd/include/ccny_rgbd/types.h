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

#ifndef CCNY_RGBD_TYPES_H
#define CCNY_RGBD_TYPES_H

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>

#include "ccny_rgbd/FeatureDetectorConfig.h"
#include "ccny_rgbd/GftDetectorConfig.h"
#include "ccny_rgbd/StarDetectorConfig.h"
#include "ccny_rgbd/SurfDetectorConfig.h"
#include "ccny_rgbd/OrbDetectorConfig.h"

namespace ccny_rgbd {

// Eigen matrix types

typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Affine3f Pose;
typedef Eigen::Affine3f AffineTransform;

// Vector types

typedef std::vector<int>             IntVector;
typedef std::vector<float>           FloatVector;
typedef std::vector<bool>            BoolVector;
typedef std::vector<cv::Point2f>     Point2fVector;
typedef std::vector<cv::Point3f>     Point3fVector;
typedef std::vector<Eigen::Matrix3f> Matrix3fVector;
typedef std::vector<Eigen::Vector3f> Vector3fVector;
typedef std::vector<cv::KeyPoint>    KeypointVector;

typedef Eigen::aligned_allocator<AffineTransform> AffineTransformAllocator; 
typedef std::vector<AffineTransform, AffineTransformAllocator> AffineTransformVector;

// PCL types

typedef pcl::PointXYZRGB              PointT;
typedef pcl::PointCloud<PointT>       PointCloudT;

typedef pcl::PointXYZ                 PointFeature;
typedef pcl::PointCloud<PointFeature> PointCloudFeature;

typedef pcl::KdTreeFLANN<PointFeature> KdTree;
typedef pcl::registration::TransformationEstimationSVD<PointFeature, PointFeature> TransformationEstimationSVD; 

// ROS message types

typedef sensor_msgs::Image            ImageMsg;
typedef sensor_msgs::CameraInfo       CameraInfoMsg;
typedef nav_msgs::Odometry            OdomMsg;
typedef nav_msgs::Path                PathMsg;

// ROS publishers, subscribers, services, etc

typedef image_geometry::PinholeCameraModel PinholeCameraModel;
typedef image_transport::ImageTransport ImageTransport;  
typedef image_transport::Publisher ImagePublisher;
typedef image_transport::SubscriberFilter ImageSubFilter;
typedef message_filters::Subscriber<CameraInfoMsg> CameraInfoSubFilter;

typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, CameraInfoMsg> RGBDSyncPolicy3;
typedef message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, CameraInfoMsg, CameraInfoMsg> RGBDSyncPolicy4;
typedef message_filters::Synchronizer<RGBDSyncPolicy3> RGBDSynchronizer3;
typedef message_filters::Synchronizer<RGBDSyncPolicy4> RGBDSynchronizer4;

// ROS dynamic reconfigure

typedef dynamic_reconfigure::Server<FeatureDetectorConfig> FeatureDetectorConfigServer;

typedef dynamic_reconfigure::Server<GftDetectorConfig> GftDetectorConfigServer;
typedef boost::shared_ptr<GftDetectorConfigServer> GftDetectorConfigServerPtr;

typedef dynamic_reconfigure::Server<StarDetectorConfig> StarDetectorConfigServer;
typedef boost::shared_ptr<StarDetectorConfigServer> StarDetectorConfigServerPtr; 

typedef dynamic_reconfigure::Server<SurfDetectorConfig> SurfDetectorConfigServer;
typedef boost::shared_ptr<SurfDetectorConfigServer> SurfDetectorConfigServerPtr; 

typedef dynamic_reconfigure::Server<OrbDetectorConfig> OrbDetectorConfigServer;
typedef boost::shared_ptr<OrbDetectorConfigServer> OrbDetectorConfigServerPtr; 

} // namespace ccny_rgbd

#endif // CCNY_RGBD_TYPES_H
