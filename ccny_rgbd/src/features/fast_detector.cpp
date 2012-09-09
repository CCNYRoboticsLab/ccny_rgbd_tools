#include "ccny_rgbd/features/fast_detector.h"

namespace ccny_rgbd
{

FastDetector::FastDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
  FeatureDetector(nh, nh_private)
{
  if (!nh_private_.getParam ("feature/FAST/threshold", threshold_))
    threshold_ = 1;
  if (!nh_private_.getParam ("feature/FAST/nonmax_suppression", nonmax_suppression_))
    nonmax_suppression_ = false;

  fast_detector_ = new cv::FastFeatureDetector(
	  threshold_, nonmax_suppression_);
}

FastDetector::~FastDetector()
{
  delete fast_detector_;
}

void FastDetector::findFeatures(RGBDFrame& frame, const cv::Mat * input_img)
{
  fast_detector_->detect(*input_img, frame.keypoints);

  if(compute_descriptors_)
  {
    orb_descriptor_.compute(*input_img, frame.keypoints, frame.descriptors);
    frame.descriptors_computed = true;
  }
}

} //namespace
