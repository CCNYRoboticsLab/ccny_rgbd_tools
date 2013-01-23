/*
 *  Copyright (C) 2013, City University of New York
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  CCNY Robotics Lab
 *  http://robotics.ccny.cuny.edu
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
