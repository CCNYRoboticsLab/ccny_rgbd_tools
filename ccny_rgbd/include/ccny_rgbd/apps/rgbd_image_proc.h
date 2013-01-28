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
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <dynamic_reconfigure/server.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/rgbd_util.h"
#include "ccny_rgbd/proc_util.h"
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

    RGBDImageProc(
      const ros::NodeHandle& nh, 
      const ros::NodeHandle& nh_private);
    
    virtual ~RGBDImageProc();

    void RGBDCallback(  
      const ImageMsg::ConstPtr& rgb_msg,
      const ImageMsg::ConstPtr& depth_msg,
      const CameraInfoMsg::ConstPtr& rgb_inf_msg,
      const CameraInfoMsg::ConstPtr& depth_info_msg);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    boost::shared_ptr<RGBDSynchronizer4> sync_;
       
    // image transport for rgb and depth
    ImageTransport rgb_image_transport_;
    ImageTransport depth_image_transport_;
    
    ImageSubFilter      sub_rgb_;
    ImageSubFilter      sub_depth_;

    CameraInfoSubFilter sub_rgb_info_;
    CameraInfoSubFilter sub_depth_info_;
    
    ImagePublisher rgb_publisher_;
    ImagePublisher depth_publisher_;
    ros::Publisher info_publisher_;
    ros::Publisher cloud_publisher_;
    
    ProcConfigServer config_server_;
    
    // parameters
    
    int queue_size_;
    
    std::string calib_path_;
    std::string calib_extr_filename_;
    std::string calib_warp_filename_;
    
    bool unwarp_;
    bool publish_cloud_;
    
    double scale_;
   
    // **** state variables
    
    bool initialized_;
    boost::mutex mutex_;
    
    // **** calibration
    
    int fit_mode_;
    cv::Size size_in_, size_out_;
    
    // depth warp polynomial coeff
    cv::Mat coeff_0_, coeff_1_, coeff_2_;   
    
    // depth warp polynomial coeff, after recitfication and resizing
    cv::Mat coeff_0_rect_, coeff_1_rect_, coeff_2_rect_;  
    
    cv::Mat ir2rgb_;    
    
    cv::Mat intr_rect_rgb_, intr_rect_depth_;  // optimal intrinsics after rectification
    
    CameraInfoMsg rgb_rect_info_msg_;   // derived from optimal matrices
    CameraInfoMsg depth_rect_info_msg_; // derived from optimal matrices
        
    cv::Mat map_rgb_1_,   map_rgb_2_;         // rectification maps
    cv::Mat map_depth_1_, map_depth_2_;
    
    void initMaps(
      const CameraInfoMsg::ConstPtr& rgb_info_msg,
      const CameraInfoMsg::ConstPtr& depth_info_msg);
    
    bool loadCalibration();   
    bool loadUnwarpCalibration();

    void reconfigCallback(ProcConfig& config, uint32_t level);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_IMAGE_PROC_H
