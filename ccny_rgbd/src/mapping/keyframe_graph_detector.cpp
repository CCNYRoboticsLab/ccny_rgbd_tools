/**
 *  @file keyframe_graph_detector.cpp
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

#include "ccny_rgbd/mapping/keyframe_graph_detector.h"

namespace ccny_rgbd {

KeyframeGraphDetector::KeyframeGraphDetector(
  const ros::NodeHandle& nh, 
  const ros::NodeHandle& nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  srand(time(NULL));

  // params
  if (!nh_private_.getParam ("graph/max_ransac_iterations", max_ransac_iterations_))
    max_ransac_iterations_ = 2000;
  if (!nh_private_.getParam ("graph/save_ransac_results", save_ransac_results_))
    save_ransac_results_ = false;
  if (!nh_private_.getParam ("graph/ransac_results_path", ransac_results_path_))
    ransac_results_path_ = std::getenv("HOME");
  if (!nh_private_.getParam ("graph/n_ransac_candidates", n_ransac_candidates_))
    n_ransac_candidates_ = 15;
  if (!nh_private_.getParam ("graph/k_nearest_neighbors", k_nearest_neighbors_))
    k_nearest_neighbors_ = 15;
  if (!nh_private_.getParam ("graph/min_ransac_inliers", min_ransac_inliers_))
    min_ransac_inliers_ = 30;
  if (!nh_private_.getParam ("graph/max_corresp_dist_desc", max_corresp_dist_desc_))
    max_corresp_dist_desc_ = 1.0;
  if (!nh_private_.getParam ("graph/max_corresp_dist_eucl", max_corresp_dist_eucl_))
    max_corresp_dist_eucl_ = 0.03;
  if (!nh_private_.getParam ("graph/n_keypoints", n_keypoints_))
    n_keypoints_ = 200;
  if (!nh_private_.getParam ("graph/init_surf_threshold", init_surf_threshold_))
    init_surf_threshold_ = 400;  

  // derived params
  max_corresp_dist_eucl_sq_ = max_corresp_dist_eucl_ * max_corresp_dist_eucl_;
}

KeyframeGraphDetector::~KeyframeGraphDetector()
{

}

int KeyframeGraphDetector::generateSingleKeyframeAssociations(
  KeyframeVector& keyframes,
  int kf_idx,
  KeyframeAssociationVector& associations)
{   
  int n_assocuations_found;
  
  ros::WallTime start= ros::WallTime::now();
  
  // calcualte neccessary features
  ros::WallTime start_features = ros::WallTime::now();
  prepareKeyframeForRANSAC(keyframes, kf_idx);  
  double dur_features = getMsDuration(start_features);
  
  // add VO association from previous to this 
  if (kf_idx > 0)
    addVisualOdometryAssociation(kf_idx-1, kf_idx, keyframes, associations);
  
  // *****************************
  //int result = singleKeyframeTreeAssociations(keyframes, kf_idx, associations);

  // get all the geometric candidates
  BoolVector candidate_keyframe_mask;
  int n_candidates = getGeometricCandidateKeyframes(keyframes, kf_idx, candidate_keyframe_mask);
    
  double dur_train, dur_assoc;
  if (n_candidates > 0)
  {
    // train matcher from features in the geometric candidates
    ros::WallTime start_train = ros::WallTime::now();  
    cv::FlannBasedMatcher matcher;
    trainMatcherFromIndices(keyframes, candidate_keyframe_mask, matcher);
    dur_train = getMsDuration(start_train);
      
    // find the aassociations from the matcher, and add them to the associations vector  
    ros::WallTime start_assoc = ros::WallTime::now();  
    n_assocuations_found = findMatcherAssociations(keyframes, kf_idx, matcher, associations);
    dur_assoc = getMsDuration(start_assoc);
  }
  else
  {
    n_assocuations_found = 0;
    dur_train = 0.0;
    dur_train = 0.0;
  }
      
  double dur_total = getMsDuration(start);   

  printf("---------------------------------------------------------------------\n"); 
  printf("SURF: %.1f Train[%d] %.1f Assoc[%d] %.1f\n", 
    dur_features, 
    dur_train, n_candidates,
    n_assocuations_found, dur_assoc);
  
  return n_assocuations_found;
}

void KeyframeGraphDetector::generateKeyframeAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  // calcualte neccessary features
  prepareKeyframesForRANSAC(keyframes);

  // inserts consecutive associations from the visual odometry
  visualOdometryAssociations(keyframes, associations);
  
  treeAssociations(keyframes, associations);
  //manualBruteForceAssociations(keyframes, associations);
}

void KeyframeGraphDetector::prepareKeyframesForRANSAC(
  KeyframeVector& keyframes)
{ 
  //ROS_INFO("preparing SURF features for RANSAC associations...\n");  

  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); kf_idx++)
    prepareKeyframeForRANSAC(keyframes, kf_idx);
}

void KeyframeGraphDetector::prepareKeyframeForRANSAC(
  KeyframeVector& keyframes,
  int kf_idx)
{
  // parameters
  double min_surf_threshold = 25;
  
  RGBDKeyframe& keyframe = keyframes[kf_idx];
  cv::SurfDescriptorExtractor extractor;
  double surf_threshold = init_surf_threshold_;

  while (surf_threshold >= min_surf_threshold)
  {
    cv::SurfFeatureDetector detector(surf_threshold);
    keyframe.keypoints.clear();
    detector.detect(keyframe.rgb_img, keyframe.keypoints);

    //ROS_INFO("[KF %d of %d] %d SURF keypoints detected (threshold: %.1f)", 
    //  (int)kf_idx, (int)keyframes.size(), 
     // (int)keyframe.keypoints.size(), surf_threshold); 

    if ((int)keyframe.keypoints.size() < n_keypoints_)
      surf_threshold /= 2.0;
    else break;
  }

  if (save_ransac_results_)
  {
    cv::Mat kp_img;
    cv::drawKeypoints(keyframe.rgb_img, keyframe.keypoints, kp_img);
    std::stringstream ss1;
    ss1 << "kp_" << kf_idx;
    cv::imwrite(ransac_results_path_ + "/" + ss1.str() + ".png", kp_img);
  }

  extractor.compute(keyframe.rgb_img, keyframe.keypoints, keyframe.descriptors);
  keyframe.computeDistributions(); 
}

void KeyframeGraphDetector::visualOdometryAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  for (unsigned int kf_idx_a = 0; kf_idx_a < keyframes.size()-1; ++kf_idx_a)
  {
    // determine second index, NO wrap-around
    unsigned int kf_idx_b = kf_idx_a + 1;

    addVisualOdometryAssociation(kf_idx_a, kf_idx_b, keyframes, associations);
  }
}

void KeyframeGraphDetector::addVisualOdometryAssociation(
  int kf_idx_a, 
  int kf_idx_b,
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  // set up the two keyframe references
  RGBDKeyframe& keyframe_a = keyframes[kf_idx_a];
  RGBDKeyframe& keyframe_b = keyframes[kf_idx_b];

  // create an association object
  KeyframeAssociation association;
  
  association.type = KeyframeAssociation::VO;    
  association.kf_idx_a = kf_idx_a;
  association.kf_idx_b = kf_idx_b;
  association.a2b = keyframe_a.pose.inverse() * keyframe_b.pose;

  associations.push_back(association);
}

void KeyframeGraphDetector::manualBruteForceAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  // params
  double max_eucl_dist    = 0.05;
  double max_desc_dist    = 10.0;
  double min_inlier_ratio = 0.75;
  double min_inliers      = 20;

  double max_eucl_dist_sq = max_eucl_dist * max_eucl_dist;

  // generate a list of all keyframe indices, for which the keyframe
  // is manually added
  std::vector<unsigned int> manual_keyframe_indices;
  
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    const RGBDKeyframe& keyframe = keyframes[kf_idx];
    if(keyframe.manually_added) 
    {
      printf("Manual keyframe: %d\n", kf_idx);
      manual_keyframe_indices.push_back(kf_idx);
    }
  }

  for (unsigned int mn_idx_a = 0; mn_idx_a < manual_keyframe_indices.size(); ++mn_idx_a)
  for (unsigned int mn_idx_b = mn_idx_a+1; mn_idx_b < manual_keyframe_indices.size(); ++mn_idx_b)
  {
    // extract the indices of the manual keyframes
    unsigned int kf_idx_a = manual_keyframe_indices[mn_idx_a];
    unsigned int kf_idx_b = manual_keyframe_indices[mn_idx_b];

    // set up the two keyframe references
    RGBDKeyframe& keyframe_a = keyframes[kf_idx_a];
    RGBDKeyframe& keyframe_b = keyframes[kf_idx_b];

    // perform ransac matching, b onto a
    std::vector<cv::DMatch> all_matches, inlier_matches;
    Eigen::Matrix4f transformation;

    pairwiseMatchingRANSAC(keyframe_a, keyframe_b, 
      max_eucl_dist_sq, max_desc_dist, min_inlier_ratio,
      all_matches, inlier_matches, transformation);

    if (inlier_matches.size() >= min_inliers)
    {
      printf("[RANSAC %d -> %d] OK   (%d / %d)\n", 
        kf_idx_a, kf_idx_b,
        (int)inlier_matches.size(), (int)all_matches.size());

      // create an association object
      KeyframeAssociation association;
      association.type = KeyframeAssociation::RANSAC;
      association.kf_idx_a = kf_idx_a;
      association.kf_idx_b = kf_idx_b;
      association.matches  = inlier_matches;
      association.a2b = tfFromEigen(transformation);
      associations.push_back(association);
    }
    else
    {
      printf("[RANSAC %d -> %d] FAIL (%d / %d)\n", 
        kf_idx_a, kf_idx_b,
        (int)inlier_matches.size(), (int)all_matches.size());
    }
  }
}

void KeyframeGraphDetector::pairwiseMatchingRANSAC(
  const RGBDFrame& frame_a, 
  const RGBDFrame& frame_b,
  double max_eucl_dist_sq, 
  double max_desc_dist,
  double sufficient_inlier_ratio,
  std::vector<cv::DMatch>& all_matches,
  std::vector<cv::DMatch>& best_inlier_matches,
  Eigen::Matrix4f& best_transformation)
{
  // constants
  int min_sample_size = 3;

  cv::FlannBasedMatcher matcher;          // for SURF
  TransformationEstimationSVD svd;

  // **** build candidate matches ***********************************
  
  // assumes detectors and distributions are computed
  // establish all matches from b to a
  matcher.match(frame_b.descriptors, frame_a.descriptors, all_matches);

  // remove bad matches - too far away in descriptor space,
  //                    - nan, too far, or cov. too big
  std::vector<cv::DMatch> candidate_matches;
  for (unsigned int m_idx = 0; m_idx < all_matches.size(); ++m_idx)
  {
    const cv::DMatch& match = all_matches[m_idx];
    int idx_b = match.queryIdx;
    int idx_a = match.trainIdx; 

    if (match.distance < max_desc_dist && 
        frame_a.kp_valid[idx_a] && 
        frame_b.kp_valid[idx_b])
    {
      candidate_matches.push_back(all_matches[m_idx]);
    }
    
    //printf("%f %s %s\n", match.distance, 
    //  frame_a.kp_valid[idx_a]?"t":"f", 
    //  frame_b.kp_valid[idx_b]?"t":"f");
  }

  int size = candidate_matches.size();

  if (size < min_sample_size) return;
  
  // **** build 3D features for SVD ********************************

  PointCloudFeature features_a, features_b;

  features_a.resize(size);
  features_b.resize(size);

  for (int m_idx = 0; m_idx < size; ++m_idx)
  {
    const cv::DMatch& match = candidate_matches[m_idx];
    int idx_b = match.queryIdx;
    int idx_a = match.trainIdx; 

    PointFeature& p_a = features_a[m_idx];
    p_a.x = frame_a.kp_means[idx_a](0,0);
    p_a.y = frame_a.kp_means[idx_a](1,0);
    p_a.z = frame_a.kp_means[idx_a](2,0);

    PointFeature& p_b = features_b[m_idx];
    p_b.x = frame_b.kp_means[idx_b](0,0);
    p_b.y = frame_b.kp_means[idx_b](1,0);
    p_b.z = frame_b.kp_means[idx_b](2,0);
  }

  // **** main RANSAC loop ****************************************
  
  int best_n_inliers = 0;
  Eigen::Matrix4f transformation; // transformation used inside loop
  
  for (int iteration = 0; iteration < max_ransac_iterations_; ++iteration)
  {   
    // generate random indices
    IntVector sample_idx;
    getRandomIndices(min_sample_size, size, sample_idx);
    
    // build initial inliers from random indices
    IntVector inlier_idx;
    std::vector<cv::DMatch> inlier_matches;

    for (unsigned int s_idx = 0; s_idx < sample_idx.size(); ++s_idx)
    {
      int m_idx = sample_idx[s_idx];
      inlier_idx.push_back(m_idx);
      inlier_matches.push_back(candidate_matches[m_idx]);
    } 
    
    // estimate transformation from minimum set of random samples
    svd.estimateRigidTransformation(
      features_b, inlier_idx,
      features_a, inlier_idx,
      transformation);

    // evaluate transformation fitness by checking distance to all points
    PointCloudFeature features_b_tf;
    pcl::transformPointCloud(features_b, features_b_tf, transformation);

    for (int m_idx = 0; m_idx < size; ++m_idx)
    {
      const PointFeature& p_a = features_a[m_idx];
      const PointFeature& p_b = features_b_tf[m_idx];

      float dist_sq = distEuclideanSq(p_a, p_b);
      
      if (dist_sq < max_eucl_dist_sq)
      {
        inlier_idx.push_back(m_idx);
        inlier_matches.push_back(candidate_matches[m_idx]);

        // reestimate transformation from all inliers
        svd.estimateRigidTransformation(
          features_b, inlier_idx,
          features_a, inlier_idx,
          transformation);
        pcl::transformPointCloud(features_b, features_b_tf, transformation);
      }
    }
    
    // check if inliers are better than the best model so far
    int n_inliers = inlier_idx.size();

    if (n_inliers > best_n_inliers)
    {
      svd.estimateRigidTransformation(
        features_b, inlier_idx,
        features_a, inlier_idx,
        transformation);

      best_n_inliers = n_inliers;
      best_transformation = transformation;
      best_inlier_matches = inlier_matches;
    }

    // check if we reached ratio termination criteria
    double inlier_ratio = (double) n_inliers / (double) size;

    if (inlier_ratio > sufficient_inlier_ratio)
      break;
  }
}

int KeyframeGraphDetector::getGeometricCandidateKeyframes(
  const KeyframeVector& keyframes,
  int kf_idx,
  BoolVector& candidate_keyframe_mask)
{
  // parameters
  double max_candidate_dist = 3.0;
  double max_candidate_angle = 90.0 * M_PI / 180.0;
  
  const RGBDKeyframe& keyframe = keyframes[kf_idx];
  
  candidate_keyframe_mask.resize(keyframes.size());
  
  int candidates_count = 0;
  // **** go through all keyframes, and find geometric cadidates
  
  for (unsigned int kf_idx_c = 0; kf_idx_c < keyframes.size(); ++kf_idx_c)
  {
    // skip self
    if ((int)kf_idx_c == kf_idx) continue;

    const RGBDKeyframe& keyframe_candidate = keyframes[kf_idx_c];

    // check the distance from keyframe to candidate
    double dist, angle;
    getTfDifference(keyframe.pose, keyframe_candidate.pose, dist, angle);

    // if distance is within threshold, add the candidate
    if (dist < max_candidate_dist && angle < max_candidate_angle)
    {
      candidate_keyframe_mask[kf_idx_c] = true;
      candidates_count++;
    }
    else
      candidate_keyframe_mask[kf_idx_c] = false;
  }  
  
  return candidates_count;
}

// trains a matcher, given a keyframe. Only keyframes which 
// have poses within a certain threshold will enter the matcher
void KeyframeGraphDetector::trainMatcherFromIndices(
  const KeyframeVector& keyframes,
  const BoolVector& candidate_keyframe_mask,
  cv::FlannBasedMatcher& matcher)
{ 
  // **** go through all candidates, and build a flann matcher
  
  //printf("Building aggregate feature vector...\n"); 
  std::vector<cv::Mat> descriptors_vector;
  
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  { 
    if (candidate_keyframe_mask[kf_idx])
    {
      const RGBDKeyframe& keyframe_candidate = keyframes[kf_idx];
      descriptors_vector.push_back(keyframe_candidate.descriptors);
    }
    else
    {
      descriptors_vector.push_back(cv::Mat());
    }
  }
  matcher.add(descriptors_vector);

  //printf("Training feature matcher...\n");
  matcher.train();
}

void KeyframeGraphDetector::trainMatcher(
  const KeyframeVector& keyframes,
  cv::FlannBasedMatcher& matcher)
{
  //printf("Building aggregate feature vector...\n"); 
  std::vector<cv::Mat> descriptors_vector;
  
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    const RGBDKeyframe& keyframe = keyframes[kf_idx];
    descriptors_vector.push_back(keyframe.descriptors);
  }
  matcher.add(descriptors_vector);

  //printf("Training feature matcher...\n");
  matcher.train();
}

int KeyframeGraphDetector::findMatcherAssociations(
  const KeyframeVector keyframes,
  int kf_idx,
  cv::FlannBasedMatcher& matcher,
  KeyframeAssociationVector& associations)
{
  // extra params
  double sufficient_ransac_inlier_ratio = 1.0;
  
  int associations_found = 0;
  
  //printf("[KF %d of %d]:\n", (int)kf_idx, (int)keyframes.size());
  const RGBDFrame& keyframe = keyframes[kf_idx];

  // find k nearest matches for each feature in the keyframe
  std::vector<std::vector<cv::DMatch> > matches_vector;
  matcher.knnMatch(keyframe.descriptors, matches_vector, k_nearest_neighbors_);

  // create empty bins vector of Pairs <count, image_index>
  std::vector<std::pair<int, int> > bins;
  bins.resize(keyframes.size());
  for (unsigned int b = 0; b < bins.size(); ++b) 
    bins[b] = std::pair<int, int>(0, b);

  // fill out bins with match indices
  for (unsigned int j = 0; j < matches_vector.size(); ++j)
  {
    std::vector<cv::DMatch>& matches = matches_vector[j];
    for (unsigned int k = 0; k < matches.size(); ++k)
    {
      bins[matches[k].imgIdx].first++;
    }
  }

  // sort - highest counts first
  std::sort(bins.begin(), bins.end(), std::greater<std::pair<int, int> >());

  // output results
  printf(" - best matches: ");
  for (int b = 0; b < std::min(n_ransac_candidates_, (int)bins.size()); ++b)
    printf("[%d(%d)] ", bins[b].second, bins[b].first);
  printf("\n");

  // **** find top X candidates
  
  printf(" - candidate matches: ");
  IntVector ransac_candidates;
  int n_ransac_candidates_found = 0;
  for (unsigned int b = 0; b < bins.size(); ++b)
  {
    unsigned int index_a = kf_idx;
    unsigned int index_b = bins[b].second;
    int corresp_count = bins[b].first;

    // test for order consistence
    // and for minimum number of keypoints
    if (index_a > index_b && 
        corresp_count >= min_ransac_inliers_)
    {
      ransac_candidates.push_back(index_b);
      ++n_ransac_candidates_found;
      printf("[%d(%d)] ", index_b, corresp_count);
    }

    if (n_ransac_candidates_found >= n_ransac_candidates_) break;
  }
  printf("\n");

  // **** test top X candidates using RANSAC

  printf(" - ransac results: ");
  for (unsigned int rc = 0; rc < ransac_candidates.size(); ++rc)
  {
    unsigned int kf_idx_a = kf_idx;
    unsigned int kf_idx_b = ransac_candidates[rc];

    const RGBDKeyframe& keyframe_a = keyframes[kf_idx_a];
    const RGBDKeyframe& keyframe_b = keyframes[kf_idx_b];

    std::vector<cv::DMatch> all_matches;
    std::vector<cv::DMatch> inlier_matches;

    // perform ransac matching, b onto a
    Eigen::Matrix4f transformation;

    pairwiseMatchingRANSAC(keyframe_a, keyframe_b, 
      max_corresp_dist_eucl_sq_, max_corresp_dist_desc_, 
      sufficient_ransac_inlier_ratio,
      all_matches, inlier_matches, transformation);

    if ((int)inlier_matches.size() >= min_ransac_inliers_)
    {
      if (save_ransac_results_)
      {
        cv::Mat img_matches;
        cv::drawMatches(keyframe_b.rgb_img, keyframe_b.keypoints, 
                        keyframe_a.rgb_img, keyframe_a.keypoints, 
                        inlier_matches, img_matches);

        std::stringstream ss1;
        ss1 << kf_idx_a << "_to_" << kf_idx_b;
        cv::imwrite(ransac_results_path_ + "/" + ss1.str() + ".png", img_matches);
      }

      printf("o");

      // create an association object
      KeyframeAssociation association;
      association.type = KeyframeAssociation::RANSAC;
      association.kf_idx_a = kf_idx_a;
      association.kf_idx_b = kf_idx_b;
      association.matches  = inlier_matches;
      association.a2b = tfFromEigen(transformation);
      associations.push_back(association);      
      
      associations_found++;
    }
    else  
      printf("x");
  }
  printf("\n");
  
  return associations_found;
}

/*
int KeyframeGraphDetector::singleKeyframeTreeAssociations(
  KeyframeVector& keyframes,
  int kf_idx,
  KeyframeAssociationVector& associations)
{  
  // get all the geometric candidates
  BoolVector candidate_keyframe_mask;
  int n_candidates = getGeometricCandidateKeyframes(
    keyframes, kf_idx, candidate_keyframe_mask);
  printf("%d geometric candidates\n", n_candidates);
  if (n_candidates == 0) return 0; ///@todo: cleaner exit when no candidates
     
  // train matcher from features in the geometric candidates
  cv::FlannBasedMatcher matcher;
  trainMatcherFromIndices(keyframes, candidate_keyframe_mask, matcher);
    
  // find the aassociations from the matcher, and add them to the associations vector  
  int assocuations_found = findMatcherAssociations(keyframes, kf_idx, matcher, associations);
  return assocuations_found;
}*/

void KeyframeGraphDetector::treeAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{ 
  // train matcher from all teh features
  cv::FlannBasedMatcher matcher;
  trainMatcher(keyframes, matcher);

  // lookup per frame
  printf("Keyframe lookups...\n");

  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
    findMatcherAssociations(keyframes, kf_idx, matcher, associations);
}

// produces k random numbers in the range [0, n).
// Monte-Carlo based random sampling
void KeyframeGraphDetector::getRandomIndices(
  int k, int n, IntVector& output)
{
  while ((int)output.size() < k)
  {
    int random_number = rand() % n;
    bool duplicate = false;    

    for (unsigned int i = 0; i < output.size(); ++i)
    {
      if (output[i] == random_number)
      {
        duplicate = true;
        break;
      }
    }

    if (!duplicate)
      output.push_back(random_number);
  }
}

} // namespace ccny_rgbd
