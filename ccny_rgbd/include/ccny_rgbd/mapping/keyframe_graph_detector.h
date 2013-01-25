/**
 *  @file keyframe_grapph_detector.h
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

#ifndef CCNY_RGBD_KEYFRAME_GRAPH_DETECTOR_H
#define CCNY_RGBD_KEYFRAME_GRAPH_DETECTOR_H

#include <boost/thread/mutex.hpp>
#include <tf/transform_datatypes.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <opencv2/nonfree/features2d.hpp>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/structures/rgbd_keyframe.h"
#include "ccny_rgbd/structures/keyframe_association.h"

namespace ccny_rgbd {

class KeyframeGraphDetector
{
  public:

    KeyframeGraphDetector(const ros::NodeHandle& nh, 
                          const ros::NodeHandle& nh_private);
    
    virtual ~KeyframeGraphDetector();

    void generateKeyframeAssociations(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

   protected:
  
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

   private:

    // parameters
    int max_ransac_iterations_;
    bool save_ransac_results_;
    std::string ransac_results_path_;

    void prepareFeaturesForRANSAC(KeyframeVector& keyframes);

    void visualOdometryAssociations(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);
    
    void treeAssociations(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);
    
    void simplifiedRingAssociations(  
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    void ringAssociations(  
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    /* tries brute force search, but only through keyframes
     * which have been manually added (manually_added= = true)
     */
    void manualBruteForceAssociations(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    double distEuclideanSq(const PointFeature& a, const PointFeature& b)
    {
      // calculate squared Euclidean distance
      float dx = a.x - b.x;
      float dy = a.y - b.y;
      float dz = a.z - b.z;
      return dx*dx + dy*dy + dz*dz;
    }

    void getRandomIndices(int k, int n, std::vector<int>& output);

    void pairwiseMatchingRANSAC(
      RGBDFrame& frame_a, RGBDFrame& frame_b,
      double max_eucl_dist_sq, 
      double max_desc_dist,
      double sufficient_inlier_ratio,
      std::vector<cv::DMatch>& all_matches,
      std::vector<cv::DMatch>& best_inlier_matches,
      Eigen::Matrix4f& best_transformation);
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_GRAPH_DETECTOR_H
