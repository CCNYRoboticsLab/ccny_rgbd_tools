/**
 *  @file keyframe_association.h
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2013, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
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

#ifndef RGBDTOOLS_KEYFRAME_ASSOCIATION_H
#define RGBDTOOLS_KEYFRAME_ASSOCIATION_H

#include <Eigen/StdVector>
#include <opencv2/features2d/features2d.hpp>

#include "rgbdtools/types.h"

namespace rgbdtools {
 
/** @brief Class representing an association between two keyframes,
 * used for graph-based pose alignement.
 * 
 * Association types:
 *  - VO: from visual odometry
 *  - RANSAC: from RANSAC-based sparse feature matching
 *  - ODOMETRY: from classical odometry sources
 */
  
class KeyframeAssociation
{
  public:
 
    /** @brief Association types
     * 
     *  - VO: from visual odometry
     *  - RANSAC: from RANSAC-based sparse feature matching
     *  - ODOMETRY: from classical odometry sources
     */
    enum Type {VO, RANSAC, ODOMETRY};  
    
    int kf_idx_a; ///< index of keyframe A
    int kf_idx_b; ///< index of keyframe B
    
    Type type;    ///< source of the association
    
    std::vector<cv::DMatch> matches; ///< for type=RANSAC, vector of RANSAC inliers mtaches
    
    /** @brief Transform between the two keyframe poses
     * 
     * The transform is expressed in A's coordinate frame.
     * A and B's poses are expressed in the fixed frame.
     * 
     * B.pose = A.pose * a2b
     */
    AffineTransform a2b;               
};

typedef Eigen::aligned_allocator<KeyframeAssociation> KeyframeAssociationAllocator;
typedef std::vector<KeyframeAssociation, KeyframeAssociationAllocator> KeyframeAssociationVector;

} // namespace rgbdtools

#endif // RGBDTOOLS_KEYFRAME_ASSOCIATION_H
