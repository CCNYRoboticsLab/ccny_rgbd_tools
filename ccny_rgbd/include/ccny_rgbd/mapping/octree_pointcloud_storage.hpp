#ifndef CCNY_RGBD_OCTREE_POINTCLOUD_STORAGE_HPP
#define CCNY_RGBD_OCTREE_POINTCLOUD_STORAGE_HPP

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
bool OctreePointCloudStorage<PointT, LeafT, OctreeT>::addPointWithReplacement(
  const PointT& point,
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

    old_point.r = (1.0-alpha_)*old_point.r + alpha_ * point.r;
    old_point.g = (1.0-alpha_)*old_point.g + alpha_ * point.g;
    old_point.b = (1.0-alpha_)*old_point.b + alpha_ * point.b;

    index = *i;
    return true;
  }
}

} // namespace octree

} // namespace pcl

#endif // CCNY_RGBD_OCTREE_POINTCLOUD_STORAGE_HPP
