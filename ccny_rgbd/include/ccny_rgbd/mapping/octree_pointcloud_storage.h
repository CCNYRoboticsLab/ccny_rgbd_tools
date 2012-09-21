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

