/**
 *  @file rgbd_image_proc.h
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

#ifndef CCNY_RGBD_RGBD_IMAGE_PROC_H
#define CCNY_RGBD_RGBD_IMAGE_PROC_H

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <dynamic_reconfigure/server.h>
#include <rgbdtools/rgbdtools.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/util.h"
#include "ccny_rgbd/RGBDImageProcConfig.h"

namespace ccny_rgbd {

/** @brief Processes the raw output of OpenNI sensors to create
 * a stream of RGB-D images.
 * 
 * The RGBDImageProc app subscribes to a stream of rgb and depth messages.
 * If performs the following operations on the images:
 *  - Scales the images down by an arbitrary factor (see \ref scale_ param)
 *  - Undistorts both the RGB and depth image
 *  - Performs unwarping on the depth image based on some polynomial model
 *  - Registers the depth image to the RGB image 
 * 
 * The app then publishes the resulting pair of RGB and depth images, together
 * with a camera info which has no distortion, and the optimal new camera matrix
 * for both images.
 */    
class RGBDImageProc 
{
  typedef RGBDImageProcConfig ProcConfig;
  typedef dynamic_reconfigure::Server<ProcConfig> ProcConfigServer;
  
  public:

    /** @brief Constructor from ROS nodehandles
     * @param nh the public nodehandle
     * @param nh_private the private nodehandle
     */  
    RGBDImageProc(
      const ros::NodeHandle& nh, 
      const ros::NodeHandle& nh_private);
    
    /** @brief Default destructor
     */
    virtual ~RGBDImageProc();

    /** @brief Main RGBD callback
     * 
     * @param rgb_msg RGB message (8UC3)
     * @param depth_msg Depth message (16UC1, in mm)
     * @param rgb_info_msg CameraInfo message, applies to RGB image
     * @param depth_info_msg CameraInfo message, applies to RGB image
     */
    void RGBDCallback(const ImageMsg::ConstPtr& rgb_msg,
                      const ImageMsg::ConstPtr& depth_msg,
                      const CameraInfoMsg::ConstPtr& rgb_info_msg,
                      const CameraInfoMsg::ConstPtr& depth_info_msg);

  private:

    ros::NodeHandle nh_;          ///< the public nodehandle
    ros::NodeHandle nh_private_;  ///< the private nodehandle

    boost::shared_ptr<RGBDSynchronizer4> sync_; ///< ROS 4-topic synchronizer
       
    ImageTransport rgb_image_transport_;    ///< ROS image transport for rgb message
    ImageTransport depth_image_transport_;  ///< ROS image transport for depth message
    
    ImageSubFilter sub_rgb_;   ///< ROS subscriber for rgb message
    ImageSubFilter sub_depth_; ///< ROS subscriber for depth message

    CameraInfoSubFilter sub_rgb_info_;   ///< ROS subscriber for rgb camera info
    CameraInfoSubFilter sub_depth_info_; ///< ROS subscriber for depth camera info
    
    ImagePublisher rgb_publisher_;      ///< ROS rgb image publisher
    ImagePublisher depth_publisher_;    ///< ROS depth image publisher
    ros::Publisher info_publisher_;     ///< ROS camera info publisher
    ros::Publisher cloud_publisher_;    ///< ROS PointCloud publisher
    
    ProcConfigServer config_server_;    ///< ROS dynamic reconfigure server
    
    // parameters
    
    int queue_size_;          ///< ROS subscriber (and publisher) queue size parameter
    
    std::string calib_path_;  ///< Path to folder where calibration files are stored
    bool verbose_;             ///< Whether to print the rectification and unwarping messages
    bool unwarp_;             ///< Whether to perform depth unwarping based on polynomial model
    bool publish_cloud_;      ///< Whether to calculate and publish the dense PointCloud
    
    /** @brief Downasampling scale (0, 1]. For example, 
     * 2.0 will result in an output image half the size of the input
     */
    double scale_;             
    
    /** @brief path to extrinsic calibration yaml file, derived
     * from \ref calib_path_ parameter
     */
    std::string calib_extr_filename_;
    
    /** @brief path to depth unwarp calibration yaml file, derived
     * from \ref calib_path_ parameter
     */
    std::string calib_warp_filename_;
   
    // **** state variables
    
    bool initialized_;      ///< whether we have initialized from the first image
    boost::mutex mutex_;    ///< state mutex
    
    // **** calibration
    
    /** @brief Depth unwwarping mode, based on different polynomial fits
     * 
     * See \ref DepthFitMode
     */
    int fit_mode_;        
    
    cv::Size size_in_; ///< Size of the incoming images. 
    
    /** @brief depth unwarp polynomial coefficient matrices */
    cv::Mat coeff_0_, coeff_1_, coeff_2_;   
    
    /** @brief depth unwarp polynomial coefficient matrices,
     * after recitfication and resizing
     */
    cv::Mat coeff_0_rect_, coeff_1_rect_, coeff_2_rect_;  
    
    /** @brief extrinsic matrix between IR and RGB camera */
    cv::Mat ir2rgb_;    
   
    /** @brief optimal intrinsics after rectification */
    cv::Mat intr_rect_rgb_, intr_rect_depth_;  
    
    /** @brief RGB CameraInfo derived from optimal matrices */
    CameraInfoMsg rgb_rect_info_msg_;   
    
    /** @brief Depth CameraInfo derived from optimal matrices */
    CameraInfoMsg depth_rect_info_msg_; 
    
    /** @brief RGB rectification maps */
    cv::Mat map_rgb_1_,   map_rgb_2_;
    
    /** @brief Depth rectification maps */
    cv::Mat map_depth_1_, map_depth_2_;
    
    /** @brief Initializes the rectification maps from CameraInfo 
     * messages
     * 
     * @param rgb_info_msg input camera info for RGB image
     * @param depth_info_msg input camera info for depth image
     */
    void initMaps(
      const CameraInfoMsg::ConstPtr& rgb_info_msg,
      const CameraInfoMsg::ConstPtr& depth_info_msg);
    
    /** @brief Loads intrinsic and extrinsic calibration 
     * info from files 
     */
    bool loadCalibration();   
    
    /** @brief Loads depth unwarp  calibration info from files 
     */
    bool loadUnwarpCalibration();

    /** @brief ROS dynamic reconfigure callback function
     */
    void reconfigCallback(ProcConfig& config, uint32_t level);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_IMAGE_PROC_H
