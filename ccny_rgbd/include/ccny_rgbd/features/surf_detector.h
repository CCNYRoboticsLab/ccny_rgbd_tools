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

#ifndef CCNY_RGBD_SURF_DETECTOR_H
#define CCNY_RGBD_SURF_DETECTOR_H

#include <opencv2/nonfree/features2d.hpp>

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

class SurfDetector: public FeatureDetector
{
  public:

    SurfDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~SurfDetector();

    void findFeatures(RGBDFrame& frame, const cv::Mat& input_img);

  private:

    double hessian_threshold_;
    bool upright_;

    cv::SurfDescriptorExtractor surf_descriptor_;
    cv::SurfFeatureDetector * surf_detector_;
};

} //namespace ccny_rgbd



#endif // CCNY_RGBD_SURF_DETECTOR_H
