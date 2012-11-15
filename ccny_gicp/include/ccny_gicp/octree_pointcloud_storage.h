#ifndef CCNY_GICP_OCTREE_POINTCLOUD_STORAGE_H
#define CCNY_GICP_OCTREE_POINTCLOUD_STORAGE_H

//#include <pcl/octree/octree_base.h>

#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_impl.h>

//#include <pcl/octree/octree_pointcloud.h>
//#include <pcl/octree/impl/octree_pointcloud.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcl
{

namespace octree
{

template<typename PointT, 
         typename LeafT = OctreeLeafDataT<int> , 
         typename OctreeT = OctreeBase<int, LeafT> >
class OctreePointCloudStorage : public OctreePointCloudSearch<PointT, LeafT, OctreeT>
{

  public:

    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef boost::shared_ptr<PointCloudT> PointCloudTPtr;
    typedef typename OctreeT::OctreeKey OctreeKey;

    OctreePointCloudStorage(double resolution = 0.05);

/*
    OctreePointCloudStorage(const OctreePointCloudStorage& other)
    {
      this->input_ = other.input_;
      this->indices_ = other.indices_; 
      this->epsilon_ = other.epsilon_;
      this->resolution_ = other.resolution_;
      this->minX_ = other.minX_;
      this->minY_ = other.minY_;
      this->minZ_ = other.minZ_;
      this->maxX_ = other.maxX_;
      this->maxY_ = other.maxY_;
      this->maxZ_ = other.maxZ_;

      this->minKeys= other.maxKeys_;

      this->boundingBoxDefined_ = other.boundingBoxDefined_;
    }
*/
    virtual ~OctreePointCloudStorage();

    bool addPointWithReplacement(const PointT& point,
                                       PointCloudTPtr cloud,
                                       int& index);

  protected:

    double alpha_;

};

} // namespace

} // namespace

#include "ccny_gicp/octree_pointcloud_storage.hpp"

#endif // CCNY_GICP_OCTREE_POINTCLOUD_STORAGE_H

