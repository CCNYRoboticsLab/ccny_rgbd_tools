/**
 *  @file orb_detector.cpp
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

#include "ccny_rgbd/features/orb_detector.h"

namespace ccny_rgbd {

OrbDetector::OrbDetector(
  const ros::NodeHandle& nh, 
  const ros::NodeHandle& nh_private):
  FeatureDetector(nh, nh_private)
{
  if (!nh_private_.getParam ("feature/ORB/n_features", n_features_))
    n_features_ = 200;
  if (!nh_private_.getParam ("feature/ORB/threshold", threshold_))
    threshold_ = 31.0;

  orb_detector_ = new cv::OrbFeatureDetector(
    n_features_, 1.2f, 8, threshold_, 0, 2, 0, 31);
}

OrbDetector::~OrbDetector()
{
  delete orb_detector_;
}

void OrbDetector::findFeatures(RGBDFrame& frame, const cv::Mat& input_img)
{
  boost::mutex::scoped_lock(mutex_);
  
  cv::Mat mask(frame.depth_img.size(), CV_8UC1);
  frame.depth_img.convertTo(mask, CV_8U);

  orb_detector_->detect(input_img, frame.keypoints, mask);

  if(compute_descriptors_)
    orb_descriptor_.compute(
      input_img, frame.keypoints, frame.descriptors);
}

void OrbDetector::setThreshold(int threshold)
{
  boost::mutex::scoped_lock(mutex_);
  
  threshold_ = threshold;

  delete orb_detector_;

  orb_detector_ = new cv::OrbFeatureDetector(
    n_features_, 1.2f, 8, threshold_, 0, 2,0, 31);
}

void OrbDetector::setNFeatures(int n_features)
{
  boost::mutex::scoped_lock(mutex_);
  
  n_features_ = n_features;

  delete orb_detector_;

  orb_detector_ = new cv::OrbFeatureDetector(
    n_features_, 1.2f, 8, threshold_, 0, 2,0, 31);
}

int OrbDetector::getThreshold() const
{
  return threshold_;
}

int OrbDetector::getNFeatures() const
{
  return n_features_;
}

} //namespace
