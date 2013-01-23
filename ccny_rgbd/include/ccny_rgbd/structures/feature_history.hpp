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

template <typename PointT>
ccny_rgbd::FeatureHistory<PointT>::FeatureHistory():
  index_(0),
  capacity_(5)
{


}

template <typename PointT>
ccny_rgbd::FeatureHistory<PointT>::~FeatureHistory()
{


}

template <typename PointT>
void ccny_rgbd::FeatureHistory<PointT>::add(const PointCloudT& cloud)
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
void ccny_rgbd::FeatureHistory<PointT>::getAll(PointCloudT& cloud)
{
  for (unsigned int i=0; i < history_.size(); ++i)
    cloud += history_[i];
}

template <typename PointT>
void ccny_rgbd::FeatureHistory<PointT>::reset()
{
  history_.clear();
  index_ = 0;
}
