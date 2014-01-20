#ifndef RGBDTOOLS_MAP_UTIL_H
#define RGBDTOOLS_MAP_UTIL_H

#include <set>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <opencv2/nonfree/features2d.hpp>

#include "rgbdtools/types.h"
#include "rgbdtools/rgbd_keyframe.h"

namespace rgbdtools {

void buildExpectedPhiHistorgtam(
  cv::Mat& histogram,
  double degrees_per_bin,
  double stdev);

void buildPhiHistogram(
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud,
  cv::Mat& histogram,
  double degrees_per_bin);

void normalizeHistogram(cv::Mat& histogram);

void shiftHistogram(
  const cv::Mat& hist_in,
  cv::Mat& hist_out,
  int bins);

bool alignHistogram(
  const cv::Mat& hist,
  const cv::Mat& hist_exp,
  double hist_resolution,
  double& best_angle);

void create8bitHistogram(
  const cv::Mat& histogram,
  cv::Mat& histogram_norm);

void createImageFromHistogram(
  const cv::Mat& histogram,
  cv::Mat& image);

void create2DProjectionImage(
  const PointCloudT& cloud, 
  cv::Mat& img,
  double min_z = -std::numeric_limits<double>::infinity(),
  double max_z =  std::numeric_limits<double>::infinity());  

void filterCloudByHeight(
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_in,
  pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_out,
  double min_z,
  double max_z);

// *** matching

/*
void buildDenseAssociationMatrix(
  const KeyframeVector& keyframes,
  cv::Mat& association_matrix);

void buildSURFAssociationMatrixBruteForce(
  const KeyframeVector& keyframes,
  cv::Mat& correspondence_matrix,
  int threshold);
  

void buildSURFAssociationMatrixTree(
  const KeyframeVector& keyframes,
  cv::Mat& correspondence_matrix,
  int k_nearest_neighbors,
  int n_candidates,
  int threshold);

void buildRANSACCorrespondenceMatrix(
  const KeyframeVector& keyframes,
  const cv::Mat& candidate_matrix,
  cv::Mat& correspondence_matrix);

void buildSURFCandidateMatrixTree(
  const cv::Mat& match_matrix,
  cv::Mat& candidate_matrix,
  int n_candidates);

void buildSURFMatchMatrixTree(
  const KeyframeVector& keyframes,
  cv::Mat& match_matrix,
  int k_nearest_neighbors);

void floatMatrixToUintMatrix(
  const cv::Mat& mat_in, 
  cv::Mat& mat_out, 
  float scale = 0);

void prepareFeaturesForRANSAC(KeyframeVector& keyframes);

void pairwiseMatchingRANSAC(
  const RGBDFrame& frame_a, const RGBDFrame& frame_b,
  double max_eucl_dist_sq, 
  double max_desc_dist,
  double sufficient_inlier_ratio,
  int max_ransac_iterations,
  std::vector<cv::DMatch>& all_matches,
  std::vector<cv::DMatch>& best_inlier_matches,
  Eigen::Matrix4f& best_transformation);
*/

void trainSURFMatcher(
  const KeyframeVector& keyframes,
  cv::FlannBasedMatcher& matcher);

void getRandomIndices(
  int k, int n, IntVector& output);

void get3RandomIndices(
  int n, std::set<int>& mask, IntVector& output);

double distEuclideanSq(const PointFeature& a, const PointFeature& b);

void makeSymmetricOR(cv::Mat mat);

void thresholdMatrix(
  const cv::Mat& mat_in, 
  cv::Mat& mat_out,
  int threshold);

void compareAssociationMatrix(
  const cv::Mat& a,
  const cv::Mat& b,
  int& false_pos,
  int& false_neg,
  int& total);


} // namespace rgbdtools

#endif // RGBDTOOLS_MAP_UTIL_H
