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

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <boost/regex.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <rgbdtools/rgbdtools.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/util.h"
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

    /** @brief Initializes all the parameters from the ROS param server
     */
    void initParams();

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
     * pcd file.
     * 
     * The resolution of the map can be controlled via the \ref pcd_map_res_
     * parameter.
     * 
     * The argument should be the path to the .pcd file
     */
    bool savePcdMapSrvCallback(
      Save::Request& request,
      Save::Response& response);
    
    /** @brief ROS callback to create an Octomap and save it to
     * file.
     * 
     * The resolution of the map can be controlled via the \ref octomap_res_
     * parameter.
     * 
     * The argument should be the path to the .bt file
     */
    bool saveOctomapSrvCallback(
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
    ros::NodeHandle nh_private_;  ///< private nodepcdhandle
    
    std::string fixed_frame_;     ///< the fixed frame (usually "odom")
    
    int queue_size_;  ///< Subscription queue size
    
    double max_range_;  ///< Maximum threshold for  range (in the z-coordinate of the camera frame)
    double max_stdev_;  ///< Maximum threshold for range (z-coordinate) standard deviation

    rgbdtools::KeyframeVector keyframes_;    ///< vector of RGBD Keyframes
    
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
    ros::Publisher path_pub_;         ///< ROS publisher for the keyframe path
    
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
    
    /** @brief ROS service to save the entire map as pcd to disk */
    ros::ServiceServer save_pcd_map_service_;
    
    /** @brief ROS service to save octomap to disk */
    ros::ServiceServer save_octomap_service_;
    
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
    double pcd_map_res_; ///< downsampling resolution of pcd map (in meters)
    double octomap_res_;  ///< tree resolution for octomap (in meters)
    double kf_dist_eps_;  ///< linear distance threshold between keyframes
    double kf_angle_eps_; ///< angular distance threshold between keyframes
    bool octomap_with_color_; ///< whetehr to save Octomaps with color info      
    double max_map_z_;   ///< maximum z (in fixed frame) when exporting maps.
          
    // state vars
    bool manual_add_;   ///< flag indicating whetehr a manual add has been requested

    int rgbd_frame_index_;

    rgbdtools::KeyframeGraphDetector graph_detector_;  ///< builds graph from the keyframes
    rgbdtools::KeyframeGraphSolverG2O graph_solver_;    ///< optimizes the graph for global alignement

    rgbdtools::KeyframeAssociationVector associations_; ///< keyframe associations that form the graph
    
    PathMsg path_msg_;    /// < contains a vector of positions of the camera (not base) pose
    
    /** @brief processes an incoming RGBD frame with a given pose,
     * and determines whether a keyframe should be inserted
     * @param frame the incoming RGBD frame (image)
     * @param pose the pose of the base frame when RGBD image was taken
     * @retval true a keyframe was inserted
     * @retval false no keyframe was inserted
     */
    bool processFrame(const rgbdtools::RGBDFrame& frame, const AffineTransform& pose);
    
    /** @brief creates a keyframe from an RGBD frame and inserts it in
     * the keyframe vector.
     * @param frame the incoming RGBD frame (image)
     * @param pose the pose of the base frame when RGBD image was taken
     */
    void addKeyframe(const rgbdtools::RGBDFrame& frame, const AffineTransform& pose);

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
    
    /** @brief Publishes all the path message
     */
    void publishPath();
    
    /** @brief Save the full map to disk as pcd
     * @param path path to save the map to
     * @retval true save was successful
     * @retval false save failed.
     */
    bool savePcdMap(const std::string& path);
           
    /** @brief Builds an pcd map from all keyframes
     * @param map_cloud the point cloud to be built
     */
    void buildPcdMap(PointCloudT& map_cloud);
                   
   /** @brief Save the full map to disk as octomap
     * @param path path to save the map to
     * @retval true save was successful
     * @retval false save failed.
     */
    bool saveOctomap(const std::string& path);
    
    /** @brief Builds an octomap octree from all keyframes
     * @param tree reference to the octomap octree
     */
    void buildOctomap(octomap::OcTree& tree);
    
    /** @brief Builds an octomap octree from all keyframes, with color
     * @param tree reference to the octomap octree
     */
    void buildColorOctomap(octomap::ColorOcTree& tree);
        
    /** @brief Convert a tf pose to octomap pose
     * @param poseTf the tf pose
     * @return octomap pose
     */
    static inline octomap::pose6d poseTfToOctomap(
      const tf::Pose& poseTf)
    {
      return octomap::pose6d(
              pointTfToOctomap(poseTf.getOrigin()),
              quaternionTfToOctomap(poseTf.getRotation()));
    }
   
    /** @brief Convert a tf point to octomap point
    * @param poseTf the tf point
    * @return octomap point
    */
    static inline octomap::point3d pointTfToOctomap(
      const tf::Point& ptTf)
    {
      return octomap::point3d(ptTf.x(), ptTf.y(), ptTf.z());
    }
   
    /** @brief Convert a tf quaternion to octomap quaternion
    * @param poseTf the tf quaternion
    * @return octomap quaternion
    */
    static inline octomath::Quaternion quaternionTfToOctomap(
      const tf::Quaternion& qTf)
    {
      return octomath::Quaternion(qTf.w(), qTf.x(), qTf.y(), qTf.z());
    }
    
    bool savePath(const std::string& filepath);
    bool savePathTUMFormat(const std::string& filepath);
    
    bool loadPath(const std::string& filepath);
    
    void updatePathFromKeyframePoses();
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_MAPPER_H
