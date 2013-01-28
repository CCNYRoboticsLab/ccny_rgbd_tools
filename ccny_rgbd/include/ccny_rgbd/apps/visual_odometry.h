/**
 *  @file visual_odometry.h
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

#ifndef CCNY_RGBD_RGBD_VISUAL_ODOMETRY_H
#define CCNY_RGBD_RGBD_VISUAL_ODOMETRY_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/rgbd_util.h"
#include "ccny_rgbd/structures/rgbd_frame.h"

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/features/orb_detector.h"
#include "ccny_rgbd/features/surf_detector.h"
#include "ccny_rgbd/features/gft_detector.h"
#include "ccny_rgbd/features/star_detector.h"

#include "ccny_rgbd/registration/motion_estimation.h"
#include "ccny_rgbd/registration/motion_estimation_icp.h"
#include "ccny_rgbd/registration/motion_estimation_icp_prob_model.h"

namespace ccny_rgbd {

/** @brief Subscribes to incoming RGBD images and outputs 
 * the position of the moving (base) frame wrt some fixed frame.
 * 
 * The class offers a selection of sparse feature detectors (GFT, ORB, SURF, STAR),
 * as well as a selection of registration algorithms. The default registration 
 * method (ICPProbModel) aligns the incoming 3D sparse features against a persistent
 * 3D feature model, which is continuously updated using a Kalman Filer.
 */  
class VisualOdometry
{
  public:

    /** @brief Constructor from ROS nodehandles
     * @param nh the public nodehandle
     * @param nh_private the private nodehandle
     */  
    VisualOdometry(const ros::NodeHandle& nh, 
                   const ros::NodeHandle& nh_private);
    
    /** @brief Default destructor
     */
    virtual ~VisualOdometry();

  private:

    // **** ROS-related

    ros::NodeHandle nh_;                ///< the public nodehandle
    ros::NodeHandle nh_private_;        ///< the private nodehandle
    tf::TransformListener tf_listener_; ///< ROS transform listener
    tf::TransformBroadcaster tf_broadcaster_; ///< ROS transform broadcaster
    ros::Publisher odom_publisher_;           ///< ROS Odometry publisher

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

    std::string fixed_frame_; ///< Fixed frame parameter
    std::string base_frame_;  ///< Moving frame parameter

    /** @brief Feature detector type parameter
     * 
     * Possible values:
     *  - GFT (default)
     *  - SURF
     *  - STAR
     *  - ORB
     */
    std::string detector_type_;
    
    /** @brief Motion estimation (registration) type parameter
     * 
     * Possible values:
     * - ICPProbModel (default) 
     * - ICP
     */
    std::string reg_type_;

    int queue_size_;  ///< Subscription queue size
    
    // **** variables

    bool initialized_; ///< Whether the init_time and b2c_ variables have been set
    int  frame_count_; ///< RGBD frame counter
    ros::Time init_time_; ///< Time of first RGBD message

    tf::Transform b2c_;  ///< Transform from the base to the camera frame, wrt base frame
    tf::Transform f2b_;  ///< Transform from the fixed to the base frame, wrt fixed frame

    FeatureDetector * feature_detector_; ///< The feature detector object

    MotionEstimation * motion_estimation_; ///< The motion estimation object
  
    // **** private functions
    
    /** @brief Main callback for RGB, Depth, and CameraInfo messages
     * 
     * @param depth_msg Depth message (16UC1, in mm)
     * @param rgb_msg RGB message (8UC3)
     * @param info_msg CameraInfo message, applies to both RGB and depth images
     */
    void imageCb(const ImageMsg::ConstPtr& depth_msg,
                 const ImageMsg::ConstPtr& rgb_msg,
                 const CameraInfoMsg::ConstPtr& info_msg);

    /** @brief Initializes all the parameters from the ROS param server
     */
    void initParams();

    /** @brief publishes the f2b_ (fixed-to-base) transform under varios
     * forms (tf, odom)
     * 
     * \todo publish also as PoseWithCovariance
     *
     * @param header header of the incoming message, used to stamp things correctly
     */
    void publishTf(const std_msgs::Header& header);

    /** @brief Caches the transform from the base frame to the camera frame
     * @param header header of the incoming message, used to stamp things correctly
     */
    bool getBaseToCameraTf(const std_msgs::Header& header);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_VISUAL_ODOMETRY_H
