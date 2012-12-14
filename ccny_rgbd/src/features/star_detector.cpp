#include "ccny_rgbd/features/star_detector.h"

namespace ccny_rgbd
{

StarDetector::StarDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
  FeatureDetector(nh, nh_private)
{
  star_detector_ = new cv::StarFeatureDetector(
    16, 15, 10, 8, 3);
}

StarDetector::~StarDetector()
{
  delete star_detector_;
}

void StarDetector::findFeatures(RGBDFrame& frame, const cv::Mat& input_img)
{
  cv::Mat mask(frame.depth_img.size(), CV_8UC1);
  frame.depth_img.convertTo(mask, CV_8U);

  star_detector_->detect(input_img, frame.keypoints, mask);
}

} //namespace
