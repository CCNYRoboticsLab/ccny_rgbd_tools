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

#ifndef CCNY_RGBD_FEATURE_DETECTOR_H
#define CCNY_RGBD_FEATURE_DETECTOR_H

#include <vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>

#include "ccny_rgbd/structures/rgbd_frame.h"
#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

class RGBDFrame;

class FeatureDetector
{
  public:

    FeatureDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~FeatureDetector();

    void findFeatures(RGBDFrame& frame);

    virtual void findFeatures(RGBDFrame& frame, const cv::Mat& input_img) = 0;

    inline int getSmooth() const;
    inline double getMaxRange() const;
    inline double getMaxStDev() const;

    inline void setSmooth(int smooth);
    inline void setMaxRange(double max_range);
    inline void setMaxStDev(double max_stdev);

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    bool compute_descriptors_;

  private:

    ros::Publisher features_publisher_;
    ros::Publisher covariances_publisher_;

    int smooth_;

    double max_range_;
    double max_stdev_;

    bool show_keypoints_;
    bool publish_features_;
    bool publish_covariances_;

    void publishCovariances(RGBDFrame& frame);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_FEATURE_DETECTOR_H
