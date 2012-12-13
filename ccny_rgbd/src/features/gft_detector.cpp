#include "ccny_rgbd/features/gft_detector.h"

namespace ccny_rgbd
{

GftDetector::GftDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
  FeatureDetector(nh, nh_private)
{
  if (!nh_private_.getParam ("feature/GFT/n_features", n_features_))
    n_features_ = 200;
  if (!nh_private_.getParam ("feature/GFT/min_distance", min_distance_))
    min_distance_ = 1.0;
  if (!nh_private_.getParam ("feature/GFT/grid_cells", grid_cells_))
    grid_cells_ = 1;

  gft_detector_ = new cv::GoodFeaturesToTrackDetector(
    n_features_, 0.01, min_distance_);

  grid_detector_ = new cv::GridAdaptedFeatureDetector(
    gft_detector_, n_features_, grid_cells_, grid_cells_);
}

GftDetector::~GftDetector()
{
  delete gft_detector_;
  delete grid_detector_;
}

void GftDetector::findFeatures(RGBDFrame& frame, const cv::Mat& input_img)
{
  cv::Mat mask(frame.depth_img.size(), CV_8UC1);
  frame.depth_img.convertTo(mask, CV_8U);

  grid_detector_->detect(input_img, frame.keypoints, mask);
}

} //namespace
