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

#ifndef RGBDTOOLS_KEYFRAME_GRAPH_DETECTOR_H
#define RGBDTOOLS_KEYFRAME_GRAPH_DETECTOR_H

#include <boost/thread/mutex.hpp>
#include <set>
#include <pcl/registration/transformation_estimation_svd.h>
#include <opencv2/nonfree/features2d.hpp>

#include "rgbdtools/types.h"
#include "rgbdtools/rgbd_keyframe.h"
#include "rgbdtools/map_util.h"
#include "rgbdtools/graph/keyframe_association.h"

namespace rgbdtools {

/** @brief Detects graph correspondences based on visual feature
 * matching between keyframes.
 */  
class KeyframeGraphDetector
{
  public:

    enum CandidateGenerationMethod
    {
      CANDIDATE_GENERATION_BRUTE_FORCE,
      CANDIDATE_GENERATION_SURF_TREE
    };

    enum PairwiseMatchingMethod
    {
      PAIRWISE_MATCHING_BFSAC,
      PAIRWISE_MATCHING_RANSAC,
    };
    
    enum PairwiseMatcherIndex
    {
      PAIRWISE_MATCHER_LINEAR,
      PAIRWISE_MATCHER_KDTREE,
    };
    
    /** @brief Constructor from ROS nodehandles
     * @param nh the public nodehandle
     * @param nh_private the private nodehandle
     */  
    KeyframeGraphDetector();
    
    /** @brief Default destructor
     */
    virtual ~KeyframeGraphDetector();

    void setVerbose(bool verbose);
    void setNCandidates(int n_candidates);
    void setKNearestNeighbors(int k_nearest_neighbors);
    void setNKeypoints(int n_keypoints);
    void setOutputPath(const std::string& output_path);
    void setSACSaveResults(bool sac_save_results);
    void setSACReestimateTf(bool sac_reestimate_tf);
    void setCandidateGenerationMethod(CandidateGenerationMethod candidate_method);
    void setPairwiseMatchingMethod(PairwiseMatchingMethod pairwsie_matching_method);
    void setPairwiseMatcherIndex(PairwiseMatcherIndex pairwsie_matcher_index);
    void setRANSACMinTemporalDistance(int graph_min_temporal_distance);
    
    void setMatcherUseDescRatioTest(bool matcher_use_desc_ratio_test);
    void setMatcherMaxDescRatio(double matcher_max_desc_ratio);
    void setMatcherMaxDescDist(double matcher_max_desc_dist);
 
    const cv::Mat getAssociationMatrix() const { return association_matrix_; }
    const cv::Mat getCandidateMatrix() const { return candidate_matrix_; }
    const cv::Mat getCorrespondenceMatrix() const { return correspondence_matrix_; }
    const cv::Mat getMatchMatrix() const { return match_matrix_; }
        
    /** Main method for generating associatuions
     * @param keyframes the input vector of RGBD keyframes
     * @param associations reference to the output vector of associations
     */
    void generateKeyframeAssociations(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    // --------------------------------------------

    void prepareFeaturesForRANSAC(KeyframeVector& keyframes);
    
    void buildAssociationMatrix(
      const KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    /** @brief checks the features of the most recently added frame against a subset of keyframes */
    void onlineLoopClosureDetector(KeyframeVector& keyframes,
  KeyframeAssociationVector& associations); 
    
    /** @brief Extracts features from a single keyframe */
    void extractFeatures(RGBDKeyframe &keyframe);

    /** @brief Trains matcher from features in a single keyframe */
    void prepareMatcher(KeyframeVector &keyframes);

   private:
       double surf_threshold;
    int edot;
    int eintegral;
    int e;

    bool verbose_;
     
    /** @brief Maximim iterations for the RANSAC test
     */
    int ransac_max_iterations_;
    
    /** @brief How many inliers are required to pass the RANSAC test.
     * 
     * If a candidate keyframe has fewer correspondences or more, 
     * it will not be eligible for a RANSAC test 
     */
    int sac_min_inliers_;
    
    bool matcher_use_desc_ratio_test_;
    
    double matcher_max_desc_ratio_;
    
    /** @brief Maximum distance (in descriptor space) between
     * two features to be considered a correspondence candidate
     */
    double matcher_max_desc_dist_;
    
    /** @brief Maximum distance squared (in Euclidean space) between
     * two features to be considered a correspondence candidate
     */
    double sac_max_eucl_dist_sq_;
    
    bool sac_reestimate_tf_;
    
    double ransac_sufficient_inlier_ratio_;
    
    double ransac_confidence_;
    double log_one_minus_ransac_confidence_;
    
    /** @brief If true, positive RANSAC results will be saved
     * to file as images with keypoint correspondences
     */
    bool sac_save_results_;
    
    /** @brief The path where to save images if sac_save_results_ is true
     */
    std::string output_path_;
    
    /** @brief For kd-tree based correspondences, how many candidate
     * keyframes will be tested agains the query keyframe using a RANSAC test
     */
    double n_candidates_;
    
    /** @brief How many nearest neighbors are requested per keypoint
     */
    int k_nearest_neighbors_;    
        
    /** @brief Number of desired keypoints to detect in each image
     */
    int n_keypoints_;
    
    double init_surf_threshold_;

    int RANSAC_min_temporal_distance_;

    bool motion_constraint_;

    void constrainMotion(Eigen::Matrix4f& motionEig);
    
    /** @brief TREE of BRUTE_FORCE */
    CandidateGenerationMethod candidate_method_;
              
    PairwiseMatchingMethod pairwise_matching_method_;
    
    PairwiseMatcherIndex pairwise_matcher_index_;
    
    //------------------------------------------
    
    /** @brief CV_8UC1, 1 if associated, 0 otherwise */
    cv::Mat association_matrix_;
    
    /** @brief CV_8UC1, 1 if candidate, 0 otherwise */
    cv::Mat candidate_matrix_;
    
    /** @brief CV_16UC1, for tree-based matching, contains number of inlier matches */
    cv::Mat correspondence_matrix_;
    
    /** @brief CV_16UC1, for tree-based matching, contains number of total matches */
    cv::Mat match_matrix_;  
    
    std::vector<cv::FlannBasedMatcher> matchers_;

    cv::FlannBasedMatcher matcher_;
    
    // ------------
    
    void buildCandidateMatrix(const KeyframeVector& keyframes);
    
    void buildMatchMatrixSurfTree(const KeyframeVector& keyframes);
      
    void buildCandidateMatrixSurfTree();

    
    void buildCorrespondenceMatrix(
      const KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);
        
    int pairwiseMatching(
      int kf_idx_q, int kf_idx_t,
      const KeyframeVector& keyframes,
      DMatchVector& best_inlier_matches,
      Eigen::Matrix4f& best_transformation);
    
    int pairwiseMatchingBFSAC(
      int kf_idx_q, int kf_idx_t,
      const KeyframeVector& keyframes,
      DMatchVector& best_inlier_matches,
      Eigen::Matrix4f& best_transformation);
    
    int pairwiseMatchingRANSAC(
      int kf_idx_q, int kf_idx_t,
      const KeyframeVector& keyframes,
      DMatchVector& best_inlier_matches,
      Eigen::Matrix4f& best_transformation);
    
    void getCandidateMatches(
      const RGBDFrame& frame_q, const RGBDFrame& frame_t,
      cv::FlannBasedMatcher& matcher,
      DMatchVector& candidate_matches);

    void prepareMatchers(
      const KeyframeVector& keyframes);

    cv::FlannBasedMatcher trainMatcher(const RGBDKeyframe& keyframe);


    
};

} // namespace rgbdtools

#endif // RGBDTOOLS_KEYFRAME_GRAPH_DETECTOR_H
