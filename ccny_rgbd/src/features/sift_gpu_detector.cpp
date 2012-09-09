#include "ccny_rgbd/features/sift_gpu_detector.h"

namespace ccny_rgbd
{

SiftGpuDetector::SiftGpuDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
  FeatureDetector(nh, nh_private)
{
/*
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
*/
}

SiftGpuDetector::~SiftGpuDetector()
{
  //delete gft_detector_;
  //delete grid_detector_;
}

void SiftGpuDetector::findFeatures(RGBDFrame& frame, const cv::Mat * input_img)
{
  //grid_detector_->detect(*input_img, frame.keypoints);
  //gft_detector_->detect(*input_img, frame.keypoints);

  cv::gpu::GpuMat image(*input_img);
  cv::gpu::GpuMat keypoints; 
  surf_gpu_detector_(image, cv::gpu::GpuMat(), keypoints);

  frame.keypoints = cv::Mat(keypoints);
}

} //namespace
