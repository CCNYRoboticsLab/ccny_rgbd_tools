/**
 *  @file surf_detector.h
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

#include "ccny_rgbd/features/surf_detector.h"

namespace ccny_rgbd {

SurfDetector::SurfDetector():FeatureDetector()
{
  surf_detector_.reset(
    new cv::SurfFeatureDetector(threshold_, 4, 2, true, false));
}

SurfDetector::~SurfDetector()
{

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

void SurfDetector::setThreshold(double threshold)
{
  threshold_ = threshold;
    
  surf_detector_.reset(
    new cv::SurfFeatureDetector(threshold_, 4, 2, true, false));
}

} // namespace ccny_rgbd
