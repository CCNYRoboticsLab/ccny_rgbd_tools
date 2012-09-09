#ifndef CCNY_RGBD_FEATURE_HISTORY_H
#define CCNY_RGBD_FEATURE_HISTORY_H

#include <vector>

#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

template <typename PointT>
class FeatureHistory
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef pcl::PointCloud<PointT>   PointCloudT;

    FeatureHistory();
    ~FeatureHistory();

    void add(const PointCloudT& cloud);
    void reset();

    inline bool isEmpty() const { return history_.empty(); }

    inline void setCapacity(unsigned int capacity) { capacity_  = capacity; }

    inline int getSize() const { return history_.size(); }

    void getAll(PointCloudT& cloud);

  private:

    unsigned int index_;
    unsigned int capacity_;
    std::vector<PointCloudT, Eigen::aligned_allocator<PointCloudT> > history_;

};

} //namespace ccny_rgbd

#include "ccny_rgbd/structures/feature_history.hpp"

#endif // CCNY_RGBD_FEATURE_HISTORY_H
