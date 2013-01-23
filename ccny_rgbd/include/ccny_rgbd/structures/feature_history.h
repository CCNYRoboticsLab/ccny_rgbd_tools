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
