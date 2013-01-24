/**
 *  @file feature_history.h
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.comm>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2013, City University of New York
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

namespace ccny_rgbd {

/** @brief Auxiliary class for the frame-to-frame ICP class. 
* 
* The class implements a ring buffer of PointClouds
*/
  
template <typename PointT>
class FeatureHistory
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef pcl::PointCloud<PointT> PointCloudT;
    typedef Eigen::aligned_allocator<PointCloudT> PointCloudTAllocator;

    /** @brief default constructor
     */
    FeatureHistory();

    /** @brief Adds a feature cloud to the buffer
     * @param cloud the cloud to be added
     */
    void add(const PointCloudT& cloud);
    
    /** @brief Clears the buffer
     */
    void reset();

    /** @brief Checks if the buffer is mepty
     *
     * @retval true  Buffer is empty
     * @retval false Buffer is not empty
     */
    inline bool isEmpty() const { return history_.empty(); }

    /** @brief Sets the capacity of the buffer
     *
     * @param capacity the desired capacity
     */
    inline void setCapacity(unsigned int capacity) { capacity_  = capacity; }

    /** @brief Get the current number of items in the buffer
     *
     * @return The current number of items in the buffer
     */
    inline int getSize() const { return history_.size(); }

    /** @brief Get all the buffer point clouds as a single aggregated point cloud
     *
     * @param cloud Reference to the PointCloud which will hold the aggregated history
     */
    void getAll(PointCloudT& cloud);

  private:

    unsigned int index_;    ///< the current index in the buffer
    unsigned int capacity_; ///< buffer capacity
    
    std::vector<PointCloudT, PointCloudTAllocator> history_; ///< the buffer of point clouds
};

} //namespace ccny_rgbd

#include "ccny_rgbd/structures/feature_history.hpp"

#endif // CCNY_RGBD_FEATURE_HISTORY_H
