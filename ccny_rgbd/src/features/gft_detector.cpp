#include "ccny_rgbd/features/gft_detector.h"

namespace ccny_rgbd
{

GftDetector::GftDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
  FeatureDetector(nh, nh_private)
{
  if (!nh_private_.getParam ("feature/GFT/n_features", n_features_))
    n_features_ = 200;
  if (!nh_private_.getParam ("feature/GFT/quality_level", quality_level_))
    quality_level_ = 0.01;
  if (!nh_private_.getParam ("feature/GFT/min_distance", min_distance_))
    min_distance_ = 1.0;

  if (!nh_private_.getParam ("feature/GFT/grid_cells", grid_cells_))
    grid_cells_ = 1;

  gft_detector_ = new cv::GoodFeaturesToTrackDetector(
    n_features_, quality_level_, min_distance_);

  grid_detector_ = new cv::GridAdaptedFeatureDetector(
    gft_detector_, n_features_, grid_cells_, grid_cells_);
}

GftDetector::~GftDetector()
{
  delete gft_detector_;
  delete grid_detector_;
}

void GftDetector::findFeatures(RGBDFrame& frame, const cv::Mat * input_img)
{
  grid_detector_->detect(*input_img, frame.keypoints);
  //gft_detector_->detect(*input_img, frame.keypoints);
}

} //namespace
