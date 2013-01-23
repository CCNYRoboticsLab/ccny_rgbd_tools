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

#ifndef CCNY_RGBD_OCTREE_POINTCLOUD_STORAGE_H
#define CCNY_RGBD_OCTREE_POINTCLOUD_STORAGE_H

#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_impl.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{

namespace octree
{

template<typename PointT, 
         typename LeafT, 
         typename OctreeT>
class OctreePointCloudStorage : public OctreePointCloudSearch<PointT, LeafT, OctreeT>
{

  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef boost::shared_ptr<PointCloudT> PointCloudTPtr;
  typedef typename OctreeT::OctreeKey OctreeKey;

  public:

    OctreePointCloudStorage(double resolution = 0.05);

    virtual ~OctreePointCloudStorage();

    bool addPointWithReplacement(const PointT& point,
                                       PointCloudTPtr cloud,
                                       int& index);

  private:

    double alpha_;
};

} // namespace

} // namespace

#include "ccny_rgbd/mapping/octree_pointcloud_storage.hpp"

#endif // CCNY_RGBD_OCTREE_POINTCLOUD_STORAGE_H

