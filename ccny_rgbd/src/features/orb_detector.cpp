#include "ccny_rgbd/features/orb_detector.h"

namespace ccny_rgbd
{

OrbDetector::OrbDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
  FeatureDetector(nh, nh_private)
{
  if (!nh_private_.getParam ("feature/ORB/n_features", n_features_))
    n_features_ = 200;
  if (!nh_private_.getParam ("feature/ORB/edge_threshold", edge_threshold_))
    edge_threshold_ = 31.0;

  orb_detector_ = new cv::OrbFeatureDetector(
	  n_features_, 1.2f, 8, edge_threshold_, 0, 2, 0, 31);
}

OrbDetector::~OrbDetector()
{
  delete orb_detector_;
}

void OrbDetector::findFeatures(RGBDFrame& frame, const cv::Mat * input_img)
{
  orb_detector_->detect(*input_img, frame.keypoints);

  if(compute_descriptors_)
  {
    orb_descriptor_.compute(*input_img, frame.keypoints, frame.descriptors);
    frame.descriptors_computed = true;
  }
}

void OrbDetector::setThreshold(int threshold)
{
  edge_threshold_ = threshold;

  delete orb_detector_;

  orb_detector_ = new cv::OrbFeatureDetector(
	  n_features_, 1.2f, 8, edge_threshold_, 0, 2,0, 31);
}

int OrbDetector::getThreshold() const
{
  return edge_threshold_;
}

void OrbDetector::setNFeatures(unsigned int n_features)
{
  n_features_ = n_features;

  delete orb_detector_;

  orb_detector_ = new cv::OrbFeatureDetector(
	  n_features_, 1.2f, 8, edge_threshold_, 0, 2,0, 31);
}

unsigned int OrbDetector::getNFeatures() const
{
  return n_features_;
}

} //namespace
