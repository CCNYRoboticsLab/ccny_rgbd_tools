#ifndef CCNY_RGBD_MONO_VO
#define CCNY_RGBD_MONO_VO
/* ======================================================================
 * mono_vo.h
 *       Final Project
 *
 *  Written by Carlos Jaramillo, Roberto Valenti and Ivan Dryanovski
 *  3D Computer Vision - CSc 83020 at CUNY GC - Prof. Stamos - (Fall 2012)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 * ======================================================================
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
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
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include "ccny_rgbd/types.h"
#include "ccny_rgbd/rgbd_util.h"
#include "ccny_rgbd/structures/rgbd_keyframe.h"

namespace ccny_rgbd
{

using namespace message_filters::sync_policies;

class MonocularVisualOdometry
{
  typedef nav_msgs::Odometry OdomMsg;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    MonocularVisualOdometry(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MonocularVisualOdometry();


  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    // **** Publishers
    ros::Publisher odom_publisher_;
    ros::Publisher pub_model_; ///< Publisher for the point cloud model (sparse map)
    ros::Publisher path_pub_;         ///< ROS publisher for the keyframe path

    /** @brief publishes the path of f2b_ (fixed-to-base) transform as an Path message
     * @param header header of the incoming message, used to stamp things correctly
     */
    void publishPath(const std_msgs::Header& header);

    boost::shared_ptr<SynchronizerMonoVO> sync_;
    image_geometry::PinholeCameraModel cam_model_;
    Matrix3f intrinsic_matrix_;

    ImageSubFilter      sub_rgb_;
    CameraInfoSubFilter sub_info_;

    // **** parameters 
    std::string pcd_filename_;
    std::string fixed_frame_; 
    std::string base_frame_;
    std::string path_to_keyframes_;
    int initial_keyframe_number_;

    std::string detector_type_;
    std::string descriptor_type_;
    
    double detector_threshold_;
    
    double max_descriptor_space_distance_;
    int image_width_; ///< Input images will be resized to this width
    int image_height_; ///< Input images will be resized to this height
    int virtual_image_width_; ///< Virtual images will be generated at this width
    int virtual_image_height_; ///< Virtual images will be generated at this height
    float scale_from_virtual_; ///< Assists on generating the virtual image shifted to left-top by a scale factor
    // **** variables
    boost::mutex::scoped_lock mutex_lock_; ///< Thread lock on subscribed input images
    bool initialized_;
    bool is_first_time_projecting_; ///< To indicate the first instance when the complete cloud model gets projected to the camera
    bool assume_initial_position_; ///< To indicate whether the assumption of known initial position of the camera pose is applied
    bool visualize_correspondences_; ///< To indicate whether correspondeces (matched points) will be vizualized in the frame
    bool debug_mode_; ///< Puts program in debug mode
    int  frame_count_;
    ros::Time init_time_;
    nav_msgs::Path path_msg_;    /// < contains a vector of positions of the camera (not base) pose

    // PnP parameters
    int max_ransac_iterations_;
    double max_reproj_error_;
    int min_inliers_count_;
    
    int virtual_image_blur_;
    int virtual_image_fill_;

    tf::Transform b2c_;
    tf::Transform f2b_;

//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    PointCloudT::Ptr model_ptr_;

    image_transport::Publisher virtual_img_pub_;

    bool publish_cloud_model_; ///< to indicate whether the model pointcloud will be published
    bool publish_virtual_img_; ///< to indicate whether the virtual image will be published

    // Topic names:
    std::string topic_cam_info_;
    std::string topic_image_;
    std::string topic_virtual_image_;

    // **** private functions
    
    void initParams();
    
    void getVirtualImageFromKeyframe(
      const PointCloudT& cloud, 
      const Matrix3f& intrinsic, 
      const tf::Transform& extrinsic_tf, 
      cv::Mat& virtual_rgb_img,
      cv::Mat& virtual_depth_img);
    
    std::string formKeyframeName(int keyframe_number, int num_of_chars);
    void generateKeyframePaths(
      const std::string& keyframe_path, 
      int keyframe_number, 
      std::string& current_keyframe_path, 
      std::string& next_keyframe_path);

    /**
     * @brief estimates current positon in the fixed frame (odom or world) based on a given dense 3D model and a monocular image
     *
     * @param model_cloud a pointer to the 3D point cloud model
     * @param mono_img the image from the monocular camer
     * @param virtual_img the virtual image generated by projecting the model onto an image plane using the currently known position
     *
     */
    void estimatePose(
      const PointCloudT::Ptr& model_cloud, 
      const cv::Mat& mono_img);

    void imageCallback(
      const ImageMsg::ConstPtr& rgb_msg, 
      const sensor_msgs::CameraInfoConstPtr& info_msg);
    

    void publishTransform(const tf::Transform &source2target_transform, const std::string& source_frame_id, const std::string& target_frame_id);
    bool getBaseToCameraTf(const std_msgs::Header& header);
    bool readPointCloudFromPCDFile(); ///< Returns true if PCD file was read successfully.
    
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_MONO_VO
