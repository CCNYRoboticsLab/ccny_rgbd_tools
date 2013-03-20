/**
 *  @file feature_detector.cpp
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

#include "ccny_rgbd/features/feature_detector.h"

namespace ccny_rgbd {

FeatureDetector::FeatureDetector():
  compute_descriptors_(true)
{

}

FeatureDetector::~FeatureDetector()
{

}

void FeatureDetector::findFeatures(RGBDFrame& frame)
{
  boost::mutex::scoped_lock(mutex_);
  
  const cv::Mat& input_img = frame.rgb_img;

  // convert from RGB to grayscale
  cv::Mat gray_img(input_img.rows, input_img.cols, CV_8UC1);
  cvtColor(input_img, gray_img, CV_BGR2GRAY);

  // blur if needed
  if(smooth_ > 0)
  {
    int blur_size = smooth_*2 + 1;
    cv::GaussianBlur(gray_img, gray_img, cv::Size(blur_size, blur_size), 0);
  }

  // find the 2D coordinates of keypoints
  findFeatures(frame, gray_img);

  // calculates the 3D position and covariance of features
  frame.computeDistributions(max_range_, max_stdev_);
}

void FeatureDetector::setSmooth(int smooth)
{
  smooth_ = smooth;
}

void FeatureDetector::setMaxRange(double max_range)
{
  max_range_ = max_range;
}

void FeatureDetector::setMaxStDev(double max_stdev)
{
  max_stdev_ = max_stdev;
}

int FeatureDetector::getSmooth() const
{
  return smooth_;
}

double FeatureDetector::getMaxRange() const
{
  return max_range_;
} 

double FeatureDetector::getMaxStDev() const
{
  return max_stdev_;
} 

} // namespace ccny_rgbd
