/**
 *  @file feature_history.hpp
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

namespace ccny_rgbd {
  
template <typename PointT>
FeatureHistory<PointT>::FeatureHistory():
  index_(0),
  capacity_(5)
{

}

template <typename PointT>
void FeatureHistory<PointT>::add(const PointCloudT& cloud)
{
  if (history_.size() < capacity_)
  {
   history_.push_back(cloud);

   ++index_;
   if (index_ >= capacity_) index_ = 0;
  }
  else if (capacity_ > 0)
  {
   history_[index_] = cloud;

   ++index_;
   if (index_ >= capacity_) index_ = 0;
  }
}

template <typename PointT>
void FeatureHistory<PointT>::getAll(PointCloudT& cloud)
{
  for (unsigned int i=0; i < history_.size(); ++i)
    cloud += history_[i];
}

template <typename PointT>
void FeatureHistory<PointT>::reset()
{
  history_.clear();
  index_ = 0;
}

} // namespace ccny_rgbd
