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

#ifndef CCNY_RGBD_RGBD_KEYFRAME_H
#define CCNY_RGBD_RGBD_KEYFRAME_H

#include <boost/filesystem.hpp>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "ccny_rgbd/structures/rgbd_frame.h"

namespace ccny_rgbd
{

class RGBDKeyframe: public RGBDFrame
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RGBDKeyframe();
  
    RGBDKeyframe(const RGBDFrame& frame);
  
    tf::Transform pose;
    PointCloudT   cloud;

    bool manually_added;

    double path_length_linear;
    double path_length_angular;

    void constructDensePointCloud(
      double max_z = 5.5,
      double max_stdev_z = 0.03);
};

typedef Eigen::aligned_allocator<RGBDKeyframe> KeyframeAllocator;
typedef std::vector<RGBDKeyframe, KeyframeAllocator> KeyframeVector;

bool saveKeyframe(
  const RGBDKeyframe& keyframe, 
  const std::string& path,
  bool in_fixed_frame);

bool loadKeyframe(
  RGBDKeyframe& keyframe, 
  const std::string& path);

bool saveKeyframes(
  const KeyframeVector& keyframes, 
  const std::string& path,
  bool in_fixed_frame = false);

bool loadKeyframes(
  KeyframeVector& keyframes, 
  const std::string& path);

} //namespace ccny_rgbd


#endif // CCNY_RGBD_RGBD_KEYFRAME_H
