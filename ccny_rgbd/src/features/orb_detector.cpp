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

OrbDetector::OrbDetector(): FeatureDetector(),
  n_features_(400),
  threshold_(31.0)
{
  mutex_.lock();
  
  orb_detector_.reset(
    new cv::OrbFeatureDetector(n_features_, 1.2f, 8, threshold_, 0, 2, 0, 31));
  
  mutex_.unlock();
}

OrbDetector::~OrbDetector()
{

}

void OrbDetector::findFeatures(RGBDFrame& frame, const cv::Mat& input_img)
{
  mutex_.lock();
  
  cv::Mat mask(frame.depth_img.size(), CV_8UC1);
  frame.depth_img.convertTo(mask, CV_8U);

  // adaptive thresholding
  double cur_threshold = threshold_;
  while (cur_threshold > 5.0)
  {
    frame.keypoints.clear();

    orb_detector_.reset(
      new cv::OrbFeatureDetector(n_features_, 1.2f, 8, cur_threshold, 0, 2, 0, 31));

    orb_detector_->detect(input_img, frame.keypoints, mask); 
    
    if (frame.keypoints.size() >= n_features_/2) break;
    
    cur_threshold = cur_threshold * 0.5;
    printf("redetecting, cur_threshold = %.1f\n", cur_threshold);
  }
    
  if(compute_descriptors_)
    orb_descriptor_.compute(
      input_img, frame.keypoints, frame.descriptors);
    
  mutex_.unlock();
}

void OrbDetector::setThreshold(int threshold)
{
  mutex_.lock();
  
  threshold_ = threshold;

  orb_detector_.reset(
    new cv::OrbFeatureDetector(n_features_, 1.2f, 8, threshold_, 0, 2, 0, 31));
  
  mutex_.unlock();
}

void OrbDetector::setNFeatures(int n_features)
{
  mutex_.lock();
  n_features_ = n_features;

  orb_detector_.reset(
    new cv::OrbFeatureDetector(n_features_, 1.2f, 8, threshold_, 0, 2, 0, 31));
  
  mutex_.unlock();
}

} //namespace
