#include "ccny_rgbd/features/surf_detector.h"

namespace ccny_rgbd
{

SurfDetector::SurfDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
  FeatureDetector(nh, nh_private)
{
  if (!nh_private_.getParam ("feature/SURF/hessian_threshold", hessian_threshold_))
    hessian_threshold_ = 400.0;
  if (!nh_private_.getParam ("feature/SURF/upright", upright_))
    upright_ = false;

  surf_detector_ = new cv::SurfFeatureDetector(
    hessian_threshold_, 4, 2, true, upright_);
}

SurfDetector::~SurfDetector()
{
  delete surf_detector_;
}

void SurfDetector::findFeatures(RGBDFrame& frame, const cv::Mat& input_img)
{
  cv::Mat mask(frame.depth_img.size(), CV_8UC1);
  frame.depth_img.convertTo(mask, CV_8U);

  surf_detector_->detect(input_img, frame.keypoints, mask);

  if(compute_descriptors_)
    surf_descriptor_.compute(
      input_img, frame.keypoints, frame.descriptors);
}

} // namespace ccny_rgbd
