#include "ccny_rgbd/loop/loop_detection.h"

namespace ccny_rgbd
{

LoopDetection::LoopDetection(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  // params

/*
  if (!nh_private_.getParam ("reg/min_feature_count", min_feature_count_))
    min_feature_count_ = 15;
  if (!nh_private_.getParam ("reg/motion_constraint", motion_constraint_ ))
    motion_constraint_  = 0;
*/

}

LoopDetection::~LoopDetection()
{

}

void LoopDetection::generateKeyframeAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  prepareFeaturesForRANSAC(keyframes);
  //ringAssociations(keyframes, associations);
  treeAssociations(keyframes, associations);
}

void LoopDetection::prepareFeaturesForRANSAC(
  KeyframeVector& keyframes)
{
  // **** clear all features and detect new ones with SURF
  printf("preparing SURF features for RANSAC associations...\n");  

  cv::SurfFeatureDetector detector(300.0);
  cv::SurfDescriptorExtractor extractor;

  //cv::OrbFeatureDetector detector(300, 1.2f, 8, 31, 0, 2, 0, 31);
  //cv::OrbDescriptorExtractor extractor;

  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    RGBDKeyframe& keyframe = keyframes[kf_idx];
    keyframe.keypoints.clear();     
    detector.detect(*keyframe.getRGBImage(), keyframe.keypoints);
    extractor.compute(*keyframe.getRGBImage(), keyframe.keypoints, keyframe.descriptors);
  }
}

void LoopDetection::ringAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  printf("Calculating RANSAC associations using ring topology...\n");  

  // **** params
  double inlier_threshold  = 0.33;
  //double matching_distance = 40.0;
  double matching_distance = 50.0;   // for orb
  double eps_reproj        = 5.0;
  bool save                = true;

  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); kf_idx++)
  { 
    // determine next index, with wraparound
    int kf_idx_a = kf_idx;
    int kf_idx_b = kf_idx + 1;
    if (kf_idx_b >= (int)keyframes.size()) 
      kf_idx_b -= (int)keyframes.size(); 

    // set up the two keyframe references
    RGBDKeyframe& frame_a = keyframes[kf_idx_a];
    RGBDKeyframe& frame_b = keyframes[kf_idx_b];

    std::vector<cv::DMatch> all_matches;
    std::vector<cv::DMatch> inlier_matches;

    getRANSACInliers(frame_a, frame_b, 
      matching_distance, eps_reproj,
      all_matches, inlier_matches);

    double inlier_ratio = (double)inlier_matches.size() / (double)all_matches.size();
    bool ransac_overlap = (inlier_ratio > inlier_threshold);

    if (ransac_overlap)
    {
      if (save)
      {
        cv::Mat img_matches;
        cv::drawMatches(*(frame_a.getRGBImage()), frame_a.keypoints, 
                        *(frame_b.getRGBImage()), frame_b.keypoints, 
                        inlier_matches, img_matches);

        std::stringstream ss1;
        ss1 << kf_idx_a << "_to_" << kf_idx_b;
        cv::imwrite("/home/idryanov/ros/images/" + ss1.str() + ".png", img_matches);
      }

      printf("RANSAC %d -> %d: PASS\n", kf_idx_a, kf_idx_b);

      // create an association object

      KeyframeAssociation association;
      association.kf_idx_a = kf_idx_a;
      association.kf_idx_b = kf_idx_b;
      association.matches  = inlier_matches;

      associations.push_back(association);
    }
    else
    {
      printf("RANSAC %d -> %d: FAIL\n", kf_idx_a, kf_idx_b);

      if (save)
      {
        cv::Mat img_matches;
        cv::drawMatches(*(frame_a.getRGBImage()), frame_a.keypoints, 
                        *(frame_b.getRGBImage()), frame_b.keypoints, 
                        all_matches, img_matches);

        std::stringstream ss1;
        ss1 << kf_idx_a << "_xx_" << kf_idx_b;
        cv::imwrite("/home/idryanov/ros/images/" + ss1.str() + ".png", img_matches);
      }

    }
  }
}

bool LoopDetection::getRANSACMatching(
  RGBDFrame& frame_src, RGBDFrame& frame_dst, 
  float matching_distance, float eps_reproj, float inlier_threshold)
{
  // **** get the matches and inlier matches
  
  std::vector<cv::DMatch> all_matches;
  std::vector<cv::DMatch> inlier_matches;

  getRANSACInliers(
    frame_src, frame_dst, 
    matching_distance, eps_reproj,
    all_matches, inlier_matches);

  // **** Check if ratio of inliers is high enough

  double inlier_ratio = (double)inlier_matches.size() / (double)all_matches.size();
  bool ransac_overlap = (inlier_ratio > inlier_threshold);

  return ransac_overlap;
}

void LoopDetection::getRANSACInliers(
  RGBDFrame& frame_src, RGBDFrame& frame_dst, 
  float matching_distance, float eps_reproj,
  std::vector<cv::DMatch>& all_matches,
  std::vector<cv::DMatch>& inlier_matches)
{
  // **** match the descriptors
  cv::FlannBasedMatcher matcher;          // for SURF
  //cv::BFMatcher matcher(cv::NORM_HAMMING);  // for ORB
  std::vector<cv::DMatch> candidate_matches;
  matcher.match(frame_src.descriptors, frame_dst.descriptors, candidate_matches);

  for (unsigned int i = 0; i < candidate_matches.size(); ++i)
  {
    if (candidate_matches[i].distance < matching_distance)
      all_matches.push_back(candidate_matches[i]);
  }

  // **** create vectors of feature points from correspondences

  std::vector<cv::Point2f> h_src_pts;
  std::vector<cv::Point2f> h_dst_pts;

  for(unsigned int i = 0; i < all_matches.size(); ++i)
  {
    int q_idx = all_matches[i].queryIdx;
    int t_idx = all_matches[i].trainIdx; 

    h_src_pts.push_back(frame_src.keypoints[q_idx].pt);
    h_dst_pts.push_back(frame_dst.keypoints[t_idx].pt);
  }

  // **** Find inliers using RANSAC

  cv::Mat status;
  cv::Mat h = cv::findHomography(h_src_pts, h_dst_pts, CV_RANSAC, eps_reproj, status);
  //cv::Mat h = cv::findHomography(h_src_pts, h_dst_pts, CV_LMEDS, eps_reproj, status);

  for (unsigned int m = 0; m < all_matches.size(); m++) 
  { 
    if (status.at<char>(m, 0) == 1)
      inlier_matches.push_back(all_matches[m]);
  }
}  

void LoopDetection::treeAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  // **** params
  double inlier_threshold  = 0.30;
  double matching_distance = 40.0;
  //double matching_distance = 50.0; // for orb
  double eps_reproj        = 5.0;

  int k_nn = 15;                        // look for X nearest neighbors
  int n_ransac_tests = 15;              // consider the first X frames with highest corr. count
  int min_corresp_for_ransac_test = 15; // if there are fewer corresp. than X, don't bother to RANSAC

  bool save = true;

  cv::FlannBasedMatcher matcher;
  //cv::BFMatcher matcher(cv::NORM_HAMMING);  // for ORB
  std::vector<cv::Mat> descriptors_vector;

  // **** build vector of keypoints matrices
  printf("Building mat vector...\n");

  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    RGBDKeyframe& keyframe = keyframes[kf_idx];
    descriptors_vector.push_back(keyframe.descriptors);
  }
  matcher.add(descriptors_vector);

  // **** build vector of keypoints matrices
  printf("Training...\n");
  matcher.train();

  // **** lookup per frame
  printf("Frame lookups...\n");

  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    printf("\tFrame %d of %d: \n", (int)kf_idx, (int)keyframes.size());
    RGBDFrame& keyframe = keyframes[kf_idx];

    // find k nearest matches for each feature in the keyframe
    std::vector<std::vector<cv::DMatch> > matches_vector;
    matcher.knnMatch(keyframe.descriptors, matches_vector, k_nn);

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

    /*
    // output results
    printf("\t\tbest matches: ");
    for (int b = 0; b < n_ransac_tests; ++b)
      printf("[%d(%d)] ", bins[b].second, bins[b].first);
    printf("\n");
    */

    // **** find top X candidates
    
    printf("\t\tcandidate matches: ");
    std::vector<int> ransac_candidates;
    int n_ransac_candidates_found = 0;
    for (unsigned int b = 0; b < bins.size(); ++b)
    {
      unsigned int index_a = kf_idx;
      unsigned int index_b = bins[b].second;
      int corresp_count = bins[b].first;

      // test for order consistence
      // (+1 to ignore consec. frames)
      // and for minimum number of keypoints
      if (index_b > index_a && 
          corresp_count >= min_corresp_for_ransac_test)
      {
        ransac_candidates.push_back(index_b);
        ++n_ransac_candidates_found;
        printf("[%d(%d)] ", index_b, corresp_count);
      }

      if (n_ransac_candidates_found >= n_ransac_tests) break;
    }
    printf("\n");

    // **** test top X candidates using RANSAC

    for (unsigned int rc = 0; rc < ransac_candidates.size(); ++rc)
    {
      unsigned int kf_idx_a = kf_idx;
      unsigned int kf_idx_b = ransac_candidates[rc];

      RGBDFrame& frame_a = keyframes[kf_idx_a];
      RGBDFrame& frame_b = keyframes[kf_idx_b];

      std::vector<cv::DMatch> all_matches;
      std::vector<cv::DMatch> inlier_matches;

      getRANSACInliers(frame_a, frame_b, 
        matching_distance, eps_reproj,
        all_matches, inlier_matches);

      double inlier_ratio = (double)inlier_matches.size() / (double)all_matches.size();
      bool ransac_overlap = (inlier_ratio > inlier_threshold);

      if (ransac_overlap)
      {
        if (save)
        {
          cv::Mat img_matches;
          cv::drawMatches(*(frame_a.getRGBImage()), frame_a.keypoints, 
                          *(frame_b.getRGBImage()), frame_b.keypoints, 
                          inlier_matches, img_matches);

          std::stringstream ss1;
          ss1 << kf_idx_a << "_to_" << kf_idx_b;
          cv::imwrite("/home/idryanov/ros/images/" + ss1.str() + ".png", img_matches);
        }

        printf("RANSAC %d -> %d: PASS\n", kf_idx_a, kf_idx_b);

        // create an association object

        KeyframeAssociation association;
        association.kf_idx_a = kf_idx_a;
        association.kf_idx_b = kf_idx_b;
        association.matches  = inlier_matches;

        associations.push_back(association);
      }
      else  
        printf("RANSAC %d -> %d: FAIL\n", kf_idx_a, kf_idx_b);
    }
  }
}

} // namespace ccny_rgbd
