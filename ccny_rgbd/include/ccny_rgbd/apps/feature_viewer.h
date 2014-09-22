/**
 *  @file feature_viewer.h
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

#ifndef CCNY_RGBD_RGBD_FEATURE_VIEWER_H
#define CCNY_RGBD_RGBD_FEATURE_VIEWER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <rgbdtools/rgbdtools.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/util.h"
#include "ccny_rgbd/FeatureDetectorConfig.h"
#include "ccny_rgbd/GftDetectorConfig.h"
#include "ccny_rgbd/StarDetectorConfig.h"
#include "ccny_rgbd/OrbDetectorConfig.h"

namespace ccny_rgbd {

/** @brief Application to test and visualize the different type of
 * feature detectors
 */  
class FeatureViewer
{ 
  public:

    /** @brief Constructor from ROS nodehandles
     * @param nh the public nodehandle
     * @param nh_private the private nodehandle
     */  
    FeatureViewer(const ros::NodeHandle& nh, 
                  const ros::NodeHandle& nh_private);
    
    /** @brief Default destructor
     */
    virtual ~FeatureViewer();

  private:

    // **** ROS-related

    ros::NodeHandle nh_;                ///< the public nodehandle
    ros::NodeHandle nh_private_;        ///< the private nodehandle

    ros::Publisher cloud_publisher_;        ///< publisher for feature point cloud
    ros::Publisher covariances_publisher_;  ///< publisher for feature covariances
    
    FeatureDetectorConfigServer config_server_; ///< ROS dynamic reconfigure server
    
    GftDetectorConfigServerPtr gft_config_server_;    ///< ROS dynamic reconfigure server for GFT params
    StarDetectorConfigServerPtr star_config_server_;  ///< ROS dynamic reconfigure server for STAR params
    OrbDetectorConfigServerPtr orb_config_server_;  ///< ROS dynamic reconfigure server for ORB params
    
    /** @brief Image transport for RGB message subscription */
    boost::shared_ptr<ImageTransport> rgb_it_;
    
    /** @brief Image transport for depth message subscription */
    boost::shared_ptr<ImageTransport> depth_it_;
    
    /** @brief Callback syncronizer */
    boost::shared_ptr<RGBDSynchronizer3> sync_;
          
    /** @brief RGB message subscriber */
    ImageSubFilter      sub_rgb_;
    
    /** @brief Depth message subscriber */
    ImageSubFilter      sub_depth_;  
   
    /** @brief Camera info message subscriber */
    CameraInfoSubFilter sub_info_;

    // **** parameters 

    /** @brief Feature detector type parameter
     * 
     * Possible values:
     *  - GFT (default)
     *  - STAR
     *  - ORB
     */
    std::string detector_type_;
  
    int queue_size_;  ///< Subscription queue size
  
     /** @brief If true, show an OpenCV window with the features
     * 
     * Note: this might slightly decrease performance
     */
    bool show_keypoints_; 
    
    /** @brief If true, publish the pcl feature cloud
     * 
     * Note: this might slightly decrease performance
     */
    bool publish_cloud_; 
    
    /** @brief If true, publish the covariance markers
     * 
     * Note: this might decrease performance
     */
    bool publish_covariances_;
 
    // **** variables

    boost::mutex mutex_; ///< state mutex
    int  frame_count_; ///< RGBD frame counter

    rgbdtools::FeatureDetectorPtr feature_detector_; ///< The feature detector object

    // **** private functions
    
    /** @brief Main callback for RGB, Depth, and CameraInfo messages
     * 
     * @param rgb_msg RGB message (8UC3)
     * @param depth_msg Depth message (16UC1, in mm)
     * @param info_msg CameraInfo message, applies to both RGB and depth images
     */
    void RGBDCallback(const ImageMsg::ConstPtr& rgb_msg,
                      const ImageMsg::ConstPtr& depth_msg,
                      const CameraInfoMsg::ConstPtr& info_msg);

    /** @brief Initializes all the parameters from the ROS param server
     */
    void initParams();
    
    /** @brief Re-instantiates the feature detector based on the detector type parameter
     */
    void resetDetector();
    
    /** @brief Publish the feature point cloud
     * 
     * Note: this might decrease performance
     */
    void publishFeatureCloud(rgbdtools::RGBDFrame& frame);
    
    /** @brief Publish the feature covariance markers
     * 
     * Note: this might decrease performance
     */
    void publishFeatureCovariances(rgbdtools::RGBDFrame& frame);
        
    /** @brief Show the image with keypoints
     * 
     * Note: this might decrease performance
     */
    void showKeypointImage(rgbdtools::RGBDFrame& frame);
    
    /** @brief ROS dynamic reconfigure callback function
     */
    void reconfigCallback(FeatureDetectorConfig& config, uint32_t level);
    
    /** @brief ROS dynamic reconfigure callback function for GFT
     */
    void gftReconfigCallback(GftDetectorConfig& config, uint32_t level);
    
    /** @brief ROS dynamic reconfigure callback function for STAR
     */
    void starReconfigCallback(StarDetectorConfig& config, uint32_t level);
    
    /** @brief ROS dynamic reconfigure callback function for ORB
     */
    void orbReconfigCallback(OrbDetectorConfig& config, uint32_t level);
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_FEATURE_VIEWER_H
