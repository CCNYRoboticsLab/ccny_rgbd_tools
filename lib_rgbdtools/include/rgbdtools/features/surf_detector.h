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

#ifndef RGBDTOOLS_SURF_DETECTOR_H
#define RGBDTOOLS_SURF_DETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/nonfree/features2d.hpp>

#include "rgbdtools/features/feature_detector.h"

namespace rgbdtools {

/** @brief SURF detector
*/  
class SurfDetector: public FeatureDetector
{
  public:

    /** @brief Default constructor
     */    
    SurfDetector();
        
    /** @brief Default destructor
     */   
    ~SurfDetector();

    /** @brief Implementation of the feature detector.
     * @param frame the input frame
     * @param input_img the image for feature detection, derived from the
     *        RGB image of the frame after (optional) blurring
     */ 
    void findFeatures(RGBDFrame& frame, const cv::Mat& input_img);

    /** @brief Set the threshold for detection
     * @param threshold threshold for detection
     */ 
    void setThreshold(double threshold);
    
  private:

    double threshold_;    ///< threshold for detection

    /** @brief OpenCV feature detector object */
    boost::shared_ptr<cv::SurfFeatureDetector> surf_detector_;     
    
    cv::SurfDescriptorExtractor surf_descriptor_; ///< OpenCV descriptor extractor object
};

typedef boost::shared_ptr<SurfDetector> SurfDetectorPtr;

} // namespace rgbdtools

#endif // RGBDTOOLS_SURF_DETECTOR_H
