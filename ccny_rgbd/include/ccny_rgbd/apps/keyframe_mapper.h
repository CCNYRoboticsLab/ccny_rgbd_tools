#ifndef CCNY_RGBD_KEYFRAME_MAPPER_H
#define CCNY_RGBD_KEYFRAME_MAPPER_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
/*
 *  Copyright (C) 2013, City University of New York
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  CCNY Robotics Lab
 *  http://robotics.ccny.cuny.edu
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

namespace ccny_rgbd
{

class KeyframeMapper
{
  public:

    KeyframeMapper(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeMapper();

    bool publishKeyframesSrvCallback(
      PublishKeyframes::Request& request,
      PublishKeyframes::Response& response);

    bool publishKeyframeSrvCallback(
      PublishKeyframe::Request& request,
      PublishKeyframe::Response& response);

    bool saveKeyframesSrvCallback(
      Save::Request& request,
      Save::Response& response);

    bool saveFullSrvCallback(
      Save::Request& request,
      Save::Response& response);

    bool loadKeyframesSrvCallback(
      Load::Request& request,
      Load::Response& response);

    bool addManualKeyframeSrvCallback(
      AddManualKeyframe::Request& request,
      AddManualKeyframe::Response& response);
    
     bool generateGraphSrvCallback(
      GenerateGraph::Request& request,
      GenerateGraph::Response& response);

    bool solveGraphSrvCallback(
      SolveGraph::Request& request,
      SolveGraph::Response& response);
    
  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    
    std::string fixed_frame_;
    
    KeyframeVector keyframes_;
    
    virtual void RGBDCallback(
      const ImageMsg::ConstPtr& depth_msg,
      const ImageMsg::ConstPtr& rgb_msg,
      const CameraInfoMsg::ConstPtr& info_msg);

  private:

    ros::Publisher keyframes_pub_;
    ros::Publisher poses_pub_;
    ros::Publisher kf_assoc_pub_;
    
    ros::ServiceServer generate_graph_service_;
    ros::ServiceServer solve_graph_service_;   
    ros::ServiceServer pub_keyframe_service_;
    ros::ServiceServer pub_keyframes_service_;
    ros::ServiceServer save_kf_service_;
    ros::ServiceServer save_full_service_;
    ros::ServiceServer load_kf_service_;
    ros::ServiceServer add_manual_keyframe_service_;

    tf::TransformListener tf_listener_;

    boost::shared_ptr<image_transport::ImageTransport> rgb_it_;
    boost::shared_ptr<image_transport::ImageTransport> depth_it_;
    boost::shared_ptr<Synchronizer> sync_;
        
    ImageSubFilter      sub_depth_;
    ImageSubFilter      sub_rgb_;
    CameraInfoSubFilter sub_info_;

    // params
    double full_map_res_;
    double kf_dist_eps_;
    double kf_angle_eps_;
          
    // state vars
    bool manual_add_;

    KeyframeGraphDetector graph_detector_;
    KeyframeGraphSolver * graph_solver_;

    KeyframeAssociationVector associations_;
    
    bool processFrame(const RGBDFrame& frame, const tf::Transform& pose);
    void addKeyframe(const RGBDFrame& frame, const tf::Transform& pose);

    void publishMatchEdge(int i, int j);
    void publishKeyframeData(int i);
    void publishKeyframePose(int i);
    
    void publishKeyframeAssociations();
    void publishKeyframePoses();
    
    bool saveFullMap(const std::string& path);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_MAPPER_H
