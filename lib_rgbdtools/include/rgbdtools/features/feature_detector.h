/**
 *  @file feature_detector.h
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

#ifndef RGBDTOOLS_FEATURE_DETECTOR_H
#define RGBDTOOLS_FEATURE_DETECTOR_H

#include <vector>
#include <boost/thread/mutex.hpp>

#include "rgbdtools/types.h"
#include "rgbdtools/rgbd_frame.h"

namespace rgbdtools {

/** @brief Base class for sparse feature extractors
 */  
class FeatureDetector
{ 
  public:

    /** @brief Default constructor
     */    
    FeatureDetector();
        
    /** @brief Default destructor
     */  
    virtual ~FeatureDetector();

    /** @brief Main function to call to detect the sparse features
     * in an RGBDFrame and fill out the corresponding information
     * @param frame the input frame
     */  
    void findFeatures(RGBDFrame& frame);

    /** @brief Returns the smoothing size.
     * 
     * Smoothing is performed using Gaussian bluring in a window of size
     * smooth*2 + 1
     * 
     * If smooth is set to 0, then no blurring will take place
     * 
     * @return smoothing window size
     */
    inline int getSmooth() const;
    
    /** @brief Returns the maximum allowed z-depth (in meters) for features
     * @return maximum allowed z-depth (in meters) for features
     */
    inline double getMaxRange() const;
    
    /** @brief Returns the maximum allowed std_dev(z) (in meters) for features
     * @return maximum allowed std_dev(z) (in meters) for features
     */
    inline double getMaxStDev() const;

     /** @brief Sets the smoothing size.
     * 
     * Smoothing is performed using Gaussian bluring in a window of size
     * smooth*2 + 1
     * 
     * If smooth is set to 0, then no blurring will take place
     * 
     * @param smooth smoothing window size
     */
    void setSmooth(int smooth);
    
    /** @brief Sets the maximum allowed z-depth (in meters) for features
     * @param max_range maximum allowed z-depth (in meters) for features
     */ 
    void setMaxRange(double max_range);
    
    /** @brief Sets the maximum allowed std_dev(z) (in meters) for features
     * @param max_stdev maximum allowed std_dev(z) (in meters) for features
     */ 
    void setMaxStDev(double max_stdev);
       
  protected:

    boost::mutex mutex_;         ///< state mutex
    
    bool compute_descriptors_;   ///< whether to calculate feature descriptors
    
    /** @brief Implementation of the feature detector.
     * @param frame the input frame
     * @param input_img the image for feature detection, derived from the
     *        RGB image of the frame after (optional) blurring
     */ 
    virtual void findFeatures(RGBDFrame& frame, const cv::Mat& input_img) = 0;
    
  private:
   
    int smooth_;        ///< blurring size (blur winddow = smooth*2 + 1)
    double max_range_;  ///< maximum allowed z-depth (in meters) for features
    double max_stdev_;  ///< maximum allowed std_dev(z) (in meters) for features
};

typedef boost::shared_ptr<FeatureDetector> FeatureDetectorPtr;

} // namespace rgbdtools

#endif // RGBDTOOLS_FEATURE_DETECTOR_H
