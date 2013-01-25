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

#ifndef CCNY_RGBD_FEATURE_DETECTOR_H
#define CCNY_RGBD_FEATURE_DETECTOR_H

#include <vector>
#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>

#include "ccny_rgbd/rgbd_util.h"
#include "ccny_rgbd/structures/rgbd_frame.h"

namespace ccny_rgbd {

/** @brief Base class for sparse feature extractors
 */  
class FeatureDetector
{
  public:

    /** @brief Constructor from ROS nodehandles
     * @param nh the public nodehandle
     * @param nh_private the private nodehandle
     */    
    FeatureDetector(const ros::NodeHandle& nh, 
                    const ros::NodeHandle& nh_private);
        
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

    /** @brief Returns the flag whether to show the keypoint image
     * @return the flag whether to show the keypoint image
     */ 
    inline bool getShowKeypoints() const;
    
    /** @brief Returns the flag whether to publish the feature point cloud
     * @return the flag whether to publish the feature point cloud
     */ 
    inline bool getPublishFeatures() const;
    
    /** @brief Sets the flag whether to publish the cavariance markers
     * @return the flag whether to publish the cavariance markers
     */ 
    inline bool getPublishCovariances() const;
    
    /** @brief Sets the smoothing size.
     * 
     * Smoothing is performed using Gaussian bluring in a window of size
     * smooth*2 + 1
     * 
     * If smooth is set to 0, then no blurring will take place
     * 
     * @param smooth smoothing window size
     */
    inline void setSmooth(int smooth);
    
    /** @brief Sets the maximum allowed z-depth (in meters) for features
     * @param max_range maximum allowed z-depth (in meters) for features
     */ 
    inline void setMaxRange(double max_range);
    
    /** @brief Sets the maximum allowed std_dev(z) (in meters) for features
     * @param max_stdev maximum allowed std_dev(z) (in meters) for features
     */ 
    inline void setMaxStDev(double max_stdev);
    
    /** @brief Sets the flag whether to show the keypoint image
     * @param show_keypoints the flag whether to show the keypoint image
     */ 
    inline void setShowKeypoints(bool show_keypoints);
    
    /** @brief Sets the flag whether to publish the feature point cloud
     * @param publish_features the flag whether to publish the feature point cloud
     */ 
    inline void setPublishFeatures(bool publish_features);
    
    /** @brief Sets the flag whether to publish the cavariance markers
     * @param publish_covariances the flag whether to publish the cavariance markers
     */ 
    inline void setPublishCovariances(bool publish_covariances);

  protected:

    ros::NodeHandle nh_;         ///< the public nodehanle
    ros::NodeHandle nh_private_; ///< the private nodehanle

    bool compute_descriptors_;   ///< whether to calculate feature descriptors

    /** @brief Implementation of the feature detector.
     * @param frame the input frame
     * @param input_img the image for feature detection, derived from the
     *        RGB image of the frame after (optional) blurring
     */ 
    virtual void findFeatures(RGBDFrame& frame, const cv::Mat& input_img) = 0;
    
  private:

    ros::Publisher features_publisher_;     ///< publisher for feature point cloud
    ros::Publisher covariances_publisher_;  ///< publisher for feature covariances

    int smooth_;      ///< blurring size (blur winddow = smooth*2 + 1)

    double max_range_;  ///< maximum allowed z-depth (in meters) for features
    double max_stdev_;  ///< maximum allowed std_dev(z) (in meters) for features

    /** @brief If true, show an OpenCV window with the features
     * 
     * Note: this might slightly decrease performance
     */
    bool show_keypoints_; 
    
    /** @brief If true, publish an OpenCV window with the
     * 
     * Note: this might slightly decrease performance
     */
    bool publish_features_; 
    
    /** @brief If true, publish the covariance markers
     * 
     * Note: this might decrease performance
     */
    bool publish_covariances_;

    /** @brief Publish the covariance markers
     * 
     * Note: this might decrease performance
     */
    void publishCovariances(RGBDFrame& frame);
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_FEATURE_DETECTOR_H
