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

#ifndef CCNY_RGBD_GFT_DETECTOR_H
#define CCNY_RGBD_GFT_DETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

class GftDetector: public FeatureDetector
{
  public:

    GftDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~GftDetector();

    void findFeatures(RGBDFrame& frame, const cv::Mat& input_img);

  private:

    int n_features_;
    double min_distance_;
    int grid_cells_;

    cv::GoodFeaturesToTrackDetector * gft_detector_;
    cv::GridAdaptedFeatureDetector * grid_detector_;
};

} //namespace ccny_rgbd



#endif // CCNY_RGBD_GFT_DETECTOR_H
