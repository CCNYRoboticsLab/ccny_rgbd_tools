/**
 *  @file keyframe_graph_detector.h
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

/** @brief Detects graph correspondences based on visual feature
 * matching between keyframes.
 */  
class KeyframeGraphDetector
{
  public:

    /** @brief Constructor from ROS nodehandles
     * @param nh the public nodehandle
     * @param nh_private the private nodehandle
     */  
    KeyframeGraphDetector(const ros::NodeHandle& nh, 
                          const ros::NodeHandle& nh_private);
    
    /** @brief Default destructor
     */
    virtual ~KeyframeGraphDetector();

    /** Main method for generating associatuions
     * @param keyframes the input vector of RGBD keyframes
     * @param associations reference to the output vector of associations
     */
    void generateKeyframeAssociations(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    int generateSingleKeyframeAssociations(
      KeyframeVector& keyframes,
      int kf_idx,
      KeyframeAssociationVector& associations);
    
   protected:
  
    ros::NodeHandle nh_;          ///< the public nodehandle
    ros::NodeHandle nh_private_;  ///< the private nodehandle

   private:

    /** @brief Maximim iterations for the RANSAC test
     */
    int max_ransac_iterations_;
    
    /** @brief If true, positive RANSAC results will be saved
     * to file as images with keypoint correspondences
     */
    bool save_ransac_results_;
    
    /** @brief The path where to save images if save_ransac_results_ is true
     */
    std::string ransac_results_path_;
    
    /** @brief For kd-tree based correspondences, how many candidate
     * keyframes will be tested agains the query keyframe using a RANSAC test
     */
    int n_ransac_candidates_;
    
    /** @brief How many nearest neighbors are requested per keypoint
     */
    int k_nearest_neighbors_;    
    
    /** @brief How many inliers are required to pass the RANSAC test.
     * 
     * If a candidate keyframe has fewer correspondences or more, 
     * it will not be eligible for a RANSAC test 
     */
    int min_ransac_inliers_;
    
    /** @brief Maximum distance (in descriptor space) between
     * two features to be considered a correspondence candidate
     */
    double max_corresp_dist_desc_;
    
    /** @brief Maximum distance (in Euclidean space) between
     * two features to be considered a correspondence candidate
     */
    double max_corresp_dist_eucl_;
    
    /** @brief Derived from max_corresp_dist_eucl_
     */
    double max_corresp_dist_eucl_sq_;
    
    /** @brief Number of desired keypoints to detect in each image
     */
    int n_keypoints_;
    
    
    /** @brief Initial SURF trheshold for detection. If not enough features 
     * are detected, threhosld is halved and detection is carried out again 
     * until threhsold drops too low or enough features are detected.
     */
    double init_surf_threshold_;

    /** @brief Goes through all the keyframes and fills out the
     * required information (features, distributinos, etc)
     * which will be needed by RANSAC matching
     *
     * Uses SURF features with an adaptive thershold to ensure a
     * minimum number of feautres (n_keypoints_) detected in each frame
     * 
     * @param keyframes the vector of keyframes to be used for associations
     */
    void prepareKeyframesForRANSAC(KeyframeVector& keyframes);

    void prepareKeyframeForRANSAC(KeyframeVector& keyframes, int kf_idx);
    
    /** @brief Creates associations based on the visual odometry poses
     * of the frames, ie, associations between consecutive frames only.
     * 
     * @param keyframes the input vector of RGBD keyframes
     * @param associations reference to the output vector of associations
     */
    void visualOdometryAssociations(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);
    
    
    void addVisualOdometryAssociation(
      int kf_idx_a, 
      int kf_idx_b,
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);
    
    
    /** @brief Creates associations based on visual matching between
     * keyframes through a RANSAC test.
     * 
     * Candidates for RANSAC testing are determined by building a kd-tree of
     * all the features (across all keyframe). Then for each keyframe, the tree
     * is used to determine a number of possible candidate keyframes. 
     * RANSAC is performed between the query frame and the valid candidates.
     * 
     * @param keyframes the input vector of RGBD keyframes
     * @param associations reference to the output vector of associations
     */
    void treeAssociations(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);   
    
    /*
    int singleKeyframeTreeAssociations(
      KeyframeVector& keyframes,
      int kf_idx,
      KeyframeAssociationVector& associations);
      */
    
    /** @brief Creates associations based on visual matching between
     * keyframes through a RANSAC test (for manually added frames)
     * 
     * Performs a brute force (each-to-each) matching between all frames which have 
     * been manually added.
     * 
     * @param keyframes the input vector of RGBD keyframes
     * @param associations reference to the output vector of associations
     */
    void manualBruteForceAssociations(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    /** @brief Squared Euclidean distance between two features in 3D
     * @param a the first 3D point
     * @param b the second 3D point
     * @return squared Euclidean distance between two a and b
     */
    double distEuclideanSq(const PointFeature& a, const PointFeature& b)
    {
      float dx = a.x - b.x;
      float dy = a.y - b.y;
      float dz = a.z - b.z;
      return dx*dx + dy*dy + dz*dz;
    }

    /** @brief Returns k distinct random numbers from 0 to (n-1)
     * @param k the number of random samples
     * @param n the (exclusive) upper limit of the number range
     * @param output the output vector of random numbers
     */
    void getRandomIndices(int k, int n, IntVector& output);

    /** @brief Aggregates all features across all keyframes and trains
     * a knn flann matcher in descriptor space
     * @param keyframes the input keyframes
     * @param matcher the reference to the matcher
     */
    void trainMatcher(const KeyframeVector& keyframes,
                      cv::FlannBasedMatcher& matcher);
    
    void trainMatcherFromIndices(
      const KeyframeVector& keyframes,
      const BoolVector& candidate_keyframe_mask,
      cv::FlannBasedMatcher& matcher);
    
    int getGeometricCandidateKeyframes(
      const KeyframeVector& keyframes,
      int kf_idx,
      BoolVector& candidate_keyframe_mask);
    
    int findMatcherAssociations(
      const KeyframeVector keyframes,
      int kf_idx,
      cv::FlannBasedMatcher& matcher,
      KeyframeAssociationVector& associations);
    
    /** @brief Given two keyframes, finds all keypoint correposndences
     * which follow a rigid transformation model
     * 
     * @param frame_a the first RGBD frame
     * @param frame_b the second RGBD frame
     * @param max_eucl_dist_sq maximum squared Euclidean distance (in meters)
     *        between two features in a correspondence for that correspondence 
     *        to be considered an inlier in the transformation model
     * @param max_desc_dist maximum descriptor distance 
     *        between two features in a correspondence for that correspondence 
     *        to be considered an inlier in the transformation model
     * @param sufficient_inlier_ratio if the ratio between inlier matches 
     *        to candidate matches exceeds this, the RANSAC test terminates
     *        successfuly
     * @param all_matches output vector of all the matches
     * @param best_inlier_matches output vectors of the matches which
     *        are inliers to the best transformation model
     * @param best_transformation the best transformation determined by RANSAC
     */
    void pairwiseMatchingRANSAC(
      const RGBDFrame& frame_a, 
      const RGBDFrame& frame_b,
      double max_eucl_dist_sq, 
      double max_desc_dist,
      double sufficient_inlier_ratio,
      std::vector<cv::DMatch>& all_matches,
      std::vector<cv::DMatch>& best_inlier_matches,
      Eigen::Matrix4f& best_transformation);
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_GRAPH_DETECTOR_H
