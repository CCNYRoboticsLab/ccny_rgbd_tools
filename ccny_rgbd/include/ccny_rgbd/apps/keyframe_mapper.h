/**
 *  @file keyframe_mapper.h
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

#ifndef CCNY_RGBD_KEYFRAME_MAPPER_H
#define CCNY_RGBD_KEYFRAME_MAPPER_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <boost/regex.hpp>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/structures/rgbd_frame.h"
#include "ccny_rgbd/structures/rgbd_keyframe.h"
#include "ccny_rgbd/mapping/keyframe_graph_detector.h"
#include "ccny_rgbd/mapping/keyframe_graph_solver_g2o.h"

#include "ccny_rgbd/GenerateGraph.h"
#include "ccny_rgbd/SolveGraph.h"
#include "ccny_rgbd/AddManualKeyframe.h"
#include "ccny_rgbd/PublishKeyframe.h"
#include "ccny_rgbd/PublishKeyframes.h"
#include "ccny_rgbd/Save.h"
#include "ccny_rgbd/Load.h"

namespace ccny_rgbd {

/** @brief Builds a 3D map from a series of RGBD keyframes.
 * 
 * The KeyframeMapper app subscribes to a stream of RGBD images, as well
 * a transform between a fixed and a moving frame (generated, for example, 
 * by the VisualOdometry app). The mapper stores a sequence of keyframes
 * containing all the necessary data to build a textured 3D point cloud map.
 * 
 * The class provides an interface to perform graph-based global alignement
 * (in post-processing).
 *
 * Additionally, the class provides an interface to save and load keyframes
 * to file, so that post-processing can be done with offline data.
 */    
class KeyframeMapper
{
  public:

    /** @brief Constructor from ROS nodehandles
     * @param nh the public nodehandle
     * @param nh_private the private nodehandle
     */  
    KeyframeMapper(const ros::NodeHandle& nh, 
                   const ros::NodeHandle& nh_private);
    
    /** @brief Default destructor
     */
    virtual ~KeyframeMapper();

    /** @brief ROS callback to publish keyframes as point clouds
     * 
     * The argument should be a regular expression string matching the
     * index of the desired keyframes to be published.
     * 
     * Note: passing a string may require escaping the quotation marks,
     * otherwise ROS sometimes confuses it for an integer
     * 
     * Examples:
     *  - /publish_keyframes ".*" --> publishes all keyframes
     *  - /publish_keyframes \"[1-9]\" --> publishes keyframes 1 to 9
     * 
     * The keyframe point clouds are published one by one.
     */
    bool publishKeyframesSrvCallback(
      PublishKeyframes::Request& request,
      PublishKeyframes::Response& response);

    /** @brief ROS callback to publish a single keyframe as point clouds
     * 
     * The argument should be an integer with the idnex of the keyframe
     */
    bool publishKeyframeSrvCallback(
      PublishKeyframe::Request& request,
      PublishKeyframe::Response& response);

    /** @brief ROS callback save all the keyframes to disk
     * 
     * The argument should be a string with the directory where to save
     * the keyframes.
     */
    bool saveKeyframesSrvCallback(
      Save::Request& request,
      Save::Response& response);

    /** @brief ROS callback to create an aggregate 3D map and save it to 
     * file.
     * 
     * The resolution of the map can be controlled via the \ref full_map_res_
     * parameter.
     * 
     * The argument should be an integer with the idnex of the keyframe
     */
    bool saveFullSrvCallback(
      Save::Request& request,
      Save::Response& response);

    /** @brief ROS callback load keyframes from disk
     * 
     * The argument should be a string with the directory pointing to 
     * the keyframes.
     */
    bool loadKeyframesSrvCallback(
      Load::Request& request,
      Load::Response& response);

    /** @brief ROS callback to manually request adding a keyframe
     */
    bool addManualKeyframeSrvCallback(
      AddManualKeyframe::Request& request,
      AddManualKeyframe::Response& response);
    
    /** @brief ROS callback to generate the graph of keyframe
     * correspondences for global alignment.
     */
     bool generateGraphSrvCallback(
      GenerateGraph::Request& request,
      GenerateGraph::Response& response);

    /** @brief ROS callback to perform global alignment
     * 
     * Note: The generate_graph service should be called prior to invoking
     * this service.
     */
    bool solveGraphSrvCallback(
      SolveGraph::Request& request,
      SolveGraph::Response& response);
    
  protected:

    ros::NodeHandle nh_;          ///< public nodehandle
    ros::NodeHandle nh_private_;  ///< private nodehandle
    
    std::string fixed_frame_;     ///< the fixed frame (usually "odom")
    
    int queue_size_;  ///< Subscription queue size
    
    KeyframeVector keyframes_;    ///< vector of RGBD Keyframes
    
    /** @brief Main callback for RGB, Depth, and CameraInfo messages
     * 
     * @param depth_msg Depth message (16UC1, in mm)
     * @param rgb_msg RGB message (8UC3)
     * @param info_msg CameraInfo message, applies to both RGB and depth images
     */
    virtual void RGBDCallback(const ImageMsg::ConstPtr& rgb_msg,
                              const ImageMsg::ConstPtr& depth_msg,
                              const CameraInfoMsg::ConstPtr& info_msg);

  private:

    ros::Publisher keyframes_pub_;    ///< ROS publisher for the keyframe point clouds
    ros::Publisher poses_pub_;        ///< ROS publisher for the keyframe poses
    ros::Publisher kf_assoc_pub_;     ///< ROS publisher for the keyframe associations
    
    /** @brief ROS service to generate the graph correpondences */
    ros::ServiceServer generate_graph_service_;
    
    /** @brief ROS service to optimize the graph */
    ros::ServiceServer solve_graph_service_;   
    
    /** @brief ROS service to publish a single keyframe */
    ros::ServiceServer pub_keyframe_service_;
    
    /** @brief ROS service to publish a subset of keyframes */
    ros::ServiceServer pub_keyframes_service_;
    
    /** @brief ROS service to save all keyframes to disk */
    ros::ServiceServer save_kf_service_;
    
    /** @brief ROS service to save the full map to disk */
    ros::ServiceServer save_full_service_;
    
    /** @brief ROS service to load all keyframes from disk */
    ros::ServiceServer load_kf_service_;
    
    /** @brief ROS service to add a manual keyframe */
    ros::ServiceServer add_manual_keyframe_service_;

    tf::TransformListener tf_listener_; ///< ROS transform listener

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
    
    // params
    double full_map_res_; ///< downsampling resolution of full map
    double kf_dist_eps_;  ///< linear distance threshold between keyframes
    double kf_angle_eps_; ///< angular distance threshold between keyframes
          
    // state vars
    bool manual_add_;   ///< flag indicating whetehr a manual add has been requested

    KeyframeGraphDetector graph_detector_;  ///< builds graph from the keyframes
    KeyframeGraphSolver * graph_solver_;    ///< optimizes the graph for global alignement

    KeyframeAssociationVector associations_; ///< keyframe associations that form the graph
    
    /** @brief processes an incoming RGBD frame with a given pose,
     * and determines whether a keyframe should be inserted
     * @param frame the incoming RGBD frame (image)
     * @param pose the pose of the base frame when RGBD image was taken
     * @retval true a keyframe was inserted
     * @retval false no keyframe was inserted
     */
    bool processFrame(const RGBDFrame& frame, const tf::Transform& pose);
    
    /** @brief creates a keyframe from an RGBD frame and inserts it in
     * the keyframe vector.
     * @param frame the incoming RGBD frame (image)
     * @param pose the pose of the base frame when RGBD image was taken
     */
    void addKeyframe(const RGBDFrame& frame, const tf::Transform& pose);

    /** @brief Publishes the point cloud associated with a keyframe
     * @param i the keyframe index
     */
    void publishKeyframeData(int i);
    
    /** @brief Publishes the pose marker associated with a keyframe
     * @param i the keyframe index
     */
    void publishKeyframePose(int i);
        
    /** @brief Publishes all the keyframe associations markers
     */
    void publishKeyframeAssociations();
    
    /** @brief Publishes all the keyframe pose markers
     */
    void publishKeyframePoses();
    
    /** @brief Save the full (downsampled) map to disk.
     * @param path path to save the map to
     * @retval true save was successful
     * @retval false save failed.
     */
    bool saveFullMap(const std::string& path);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_MAPPER_H
