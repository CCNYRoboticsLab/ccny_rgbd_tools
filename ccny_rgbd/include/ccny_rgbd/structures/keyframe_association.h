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

#ifndef CCNY_RGBD_KEYFRAME_ASSOCIATION_H
#define CCNY_RGBD_KEYFRAME_ASSOCIATION_H

#include <Eigen/StdVector>
#include <opencv2/features2d/features2d.hpp>

namespace ccny_rgbd
{

enum KeyframeAssociationType {VO, RANSAC, ODOMETRY};  
  
class KeyframeAssociation
{
  public:
 
    enum Type {VO, RANSAC, ODOMETRY};  
    
    int kf_idx_a;                    // index of keyframe A
    int kf_idx_b;                    // index of keyframe B
    
    Type type;    // source of the association
    
    std::vector<cv::DMatch> matches; // for type=RANSAC, vector of RANSAC inliers
    
    tf::Transform a2b;               // ???
};

typedef Eigen::aligned_allocator<KeyframeAssociation> KeyframeAssociationAllocator;
typedef std::vector<KeyframeAssociation, KeyframeAssociationAllocator> KeyframeAssociationVector;

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_ASSOCIATION_H
