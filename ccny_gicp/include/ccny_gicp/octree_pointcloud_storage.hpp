#ifndef CCNY_GICP_OCTREE_POINTCLOUD_STORAGE_HPP
#define CCNY_GICP_OCTREE_POINTCLOUD_STORAGE_HPP

#include "ccny_gicp/octree_pointcloud_storage.h"

namespace pcl
{

namespace octree
{

template <typename PointT, typename LeafT, typename OctreeT>
OctreePointCloudStorage<PointT, LeafT, OctreeT>::OctreePointCloudStorage(double resolution):
  OctreePointCloudSearch<PointT, LeafT, OctreeT>(resolution)
{
  alpha_ = 0.01;
}

template<typename PointT, typename LeafT, typename OctreeT>
OctreePointCloudStorage<PointT, LeafT, OctreeT>::~OctreePointCloudStorage()
{

}

template<typename PointT, typename LeafT, typename OctreeT>
bool OctreePointCloudStorage<PointT, LeafT, OctreeT>::addPointWithReplacement(const PointT& point,
                                                                                    PointCloudTPtr cloud,
                                                                                    int& index)
{
  // generate key
  OctreeKey key;     
  genOctreeKeyforPoint (point, key);

  LeafT* leaf = findLeaf(key);

  if (leaf == 0)
  {
    // not found - create new one

    addPointToCloud(point, cloud);

    index = cloud->points.size() - 1;
    return false;
  }
  else
  {
    // found - update

    const int* i;
    leaf->getData(i);
  
    PointT& old_point = cloud->points[*i];

    // option 1 - replace
    //old_point = point;

/*
    // option 2 - update through alpha 
    uint32_t old_rgb = *reinterpret_cast<uint32_t*>(&old_point.rgb);
    uint8_t old_r = (old_rgb >> 16) & 0x0000ff;
    uint8_t old_g = (old_rgb >> 8)  & 0x0000ff;
    uint8_t old_b = (old_rgb)       & 0x0000ff;

    uint32_t new_rgb = *reinterpret_cast<uint32_t*>(&point.rgb);
    uint8_t new_r = (new_rgb >> 16) & 0x0000ff;
    uint8_t new_g = (new_rgb >> 8)  & 0x0000ff;
    uint8_t new_b = (new_rgb)       & 0x0000ff;

    uint32_t upd_r = old_point*(float)new_r + (1.0-alpha_)*(float)old_r;
    uint32_t upd_g = alpha_*(float)new_g + (1.0-alpha_)*(float)old_g;  
    uint32_t upd_b = alpha_*(float)new_b + (1.0-alpha_)*(float)old_b;    

    uint32_t upd_color = (upd_r << 16) + (upd_g << 8) + upd_b;
    old_point.rgb = *reinterpret_cast<float*>(&upd_color);
*/

    old_point.r = (1.0-alpha_)*old_point.r + alpha_ * point.r;
    old_point.g = (1.0-alpha_)*old_point.g + alpha_ * point.g;
    old_point.b = (1.0-alpha_)*old_point.b + alpha_ * point.b;

    index = *i;
    return true;
  }
}

} // namespace octree

} // namespace pcl

#endif // CCNY_GICP_OCTREE_POINTCLOUD_STORAGE_HPP
