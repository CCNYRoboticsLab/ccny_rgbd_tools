/**
 *  @file gft_detector.h
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

#ifndef RGBDTOOLS_GFT_DETECTOR_H
#define RGBDTOOLS_GFT_DETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "rgbdtools/features/feature_detector.h"

namespace rgbdtools {

/** @brief GoodFeaturesToTrack detector
 */  
class GftDetector: public FeatureDetector
{ 
  public:

    /** @brief Default constructor
     */     
    GftDetector();
    
    /** @brief Default destructor
     */    
    ~GftDetector();

    /** @brief Implementation of the feature detector.
     * @param frame the input frame
     * @param input_img the image for feature detection, derived from the
     *        RGB image of the frame after (optional) blurring
     */ 
    void findFeatures(RGBDFrame& frame, const cv::Mat& input_img);

    /** @brief Set the desired number of features
     * @param n_features desired number of features
     */ 
    void setNFeatures(int n_features);
    
    /** @brief Set the minimum distance (in pixels) between the features
     * @param min_distance minimum distance (in pixels) between the features
     */ 
    void setMinDistance(double min_distance);    
    
  private:
   
    int n_features_;      ///< the number of desired features
    double min_distance_; ///< the minimum distance (in pixels) between the features

    boost::shared_ptr<cv::GoodFeaturesToTrackDetector> gft_detector_; ///< OpenCV GTF detector object   
};

typedef boost::shared_ptr<GftDetector> GftDetectorPtr;

} // namespace rgbdtools

#endif // RGBDTOOLS_GFT_DETECTOR_H
