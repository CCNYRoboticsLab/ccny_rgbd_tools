/**
 *  @file keyframe_mapper.cpp
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

#include "ccny_rgbd/apps/keyframe_mapper.h"

namespace ccny_rgbd {

KeyframeMapper::KeyframeMapper(
  const ros::NodeHandle& nh, 
  const ros::NodeHandle& nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  graph_detector_(nh_, nh_private_)
{
  ROS_INFO("Starting RGBD Keyframe Mapper");
 
  // **** init variables

  graph_solver_ = new KeyframeGraphSolverG2O(nh, nh_private);
  
  // **** params

  if (!nh_private_.getParam ("queue_size", queue_size_))
    queue_size_ = 5;
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("full_map_res", full_map_res_))
    full_map_res_ = 0.01;
  if (!nh_private_.getParam ("octomap_res", octomap_res_))
    octomap_res_ = 0.05;  
  if (!nh_private_.getParam ("kf_dist_eps", kf_dist_eps_))
    kf_dist_eps_  = 0.10;
  if (!nh_private_.getParam ("kf_angle_eps", kf_angle_eps_))
    kf_angle_eps_  = 10.0 * M_PI / 180.0;
    
  // **** publishers

  keyframes_pub_ = nh_.advertise<PointCloudT>(
    "keyframes", queue_size_);
  poses_pub_ = nh_.advertise<visualization_msgs::Marker>( 
    "keyframe_poses", queue_size_);
  kf_assoc_pub_ = nh_.advertise<visualization_msgs::Marker>( 
    "keyframe_associations", queue_size_);
  
  // **** services

  pub_keyframe_service_ = nh_.advertiseService(
    "publish_keyframe", &KeyframeMapper::publishKeyframeSrvCallback, this);
  pub_keyframes_service_ = nh_.advertiseService(
    "publish_keyframes", &KeyframeMapper::publishKeyframesSrvCallback, this);
  save_kf_service_ = nh_.advertiseService(
    "save_keyframes", &KeyframeMapper::saveKeyframesSrvCallback, this);
  load_kf_service_ = nh_.advertiseService(
    "load_keyframes", &KeyframeMapper::loadKeyframesSrvCallback, this);
  save_full_service_ = nh_.advertiseService(
    "save_full_map", &KeyframeMapper::saveFullSrvCallback, this);
  save_octomap_service_ = nh_.advertiseService(
    "save_octomap", &KeyframeMapper::saveOctomapSrvCallback, this);
  add_manual_keyframe_service_ = nh_.advertiseService(
    "add_manual_keyframe", &KeyframeMapper::addManualKeyframeSrvCallback, this);
  generate_graph_service_ = nh_.advertiseService(
    "generate_graph", &KeyframeMapper::generateGraphSrvCallback, this);
   solve_graph_service_ = nh_.advertiseService(
    "solve_graph", &KeyframeMapper::solveGraphSrvCallback, this);
 
  // **** subscribers

  ImageTransport rgb_it(nh_);
  ImageTransport depth_it(nh_);

  sub_rgb_.subscribe(rgb_it,     "/rgbd/rgb",   queue_size_);
  sub_depth_.subscribe(depth_it, "/rgbd/depth", queue_size_);
  sub_info_.subscribe(nh_,       "/rgbd/info",  queue_size_);

  // Synchronize inputs.
  sync_.reset(new RGBDSynchronizer3(
                RGBDSyncPolicy3(queue_size_), sub_rgb_, sub_depth_, sub_info_));
   
  sync_->registerCallback(boost::bind(&KeyframeMapper::RGBDCallback, this, _1, _2, _3));  
}

KeyframeMapper::~KeyframeMapper()
{
  delete graph_solver_;
}
  
void KeyframeMapper::RGBDCallback(
  const ImageMsg::ConstPtr& rgb_msg,
  const ImageMsg::ConstPtr& depth_msg,
  const CameraInfoMsg::ConstPtr& info_msg)
{
  tf::StampedTransform transform;

  const ros::Time& time = rgb_msg->header.stamp;

  try{
    tf_listener_.waitForTransform(
     fixed_frame_, rgb_msg->header.frame_id, time, ros::Duration(0.1));
    tf_listener_.lookupTransform(
      fixed_frame_, rgb_msg->header.frame_id, time, transform);  
  }
  catch(...)
  {
    return;
  }
  RGBDFrame frame(rgb_msg, depth_msg, info_msg);
  bool result = processFrame(frame, transform);
  if (result) publishKeyframeData(keyframes_.size() - 1);
}

bool KeyframeMapper::processFrame(
  const RGBDFrame& frame, 
  const tf::Transform& pose)
{
  bool result; // if true, add new frame

  if(keyframes_.empty() || manual_add_)
  {
    result = true;
  }
  else
  {
    double dist, angle;
    getTfDifference(pose, keyframes_.back().pose, dist, angle);

    if (dist > kf_dist_eps_ || angle > kf_angle_eps_)
      result = true;
    else 
      result = false;
  }

  if (result) addKeyframe(frame, pose);
  return result;
}

void KeyframeMapper::addKeyframe(
  const RGBDFrame& frame, 
  const tf::Transform& pose)
{
  //ROS_INFO("Adding frame");
  RGBDKeyframe keyframe(frame);
  keyframe.pose = pose;
  keyframe.constructDensePointCloud();

  if (manual_add_)
  {
    ROS_INFO("Adding frame manually");
    manual_add_ = false;
    keyframe.manually_added = true;
  }
  keyframes_.push_back(keyframe);
}

bool KeyframeMapper::publishKeyframeSrvCallback(
  PublishKeyframe::Request& request,
  PublishKeyframe::Response& response)
{
  int kf_idx = request.id;
  
  if (kf_idx >= 0 && kf_idx < (int)keyframes_.size())
  {
    ROS_INFO("Publishing keyframe %d", kf_idx);
    publishKeyframeData(kf_idx);
    publishKeyframePose(kf_idx);
    return true;
  }
  else
  {
    ROS_ERROR("Index out of range");
    return false;  
  }
}

bool KeyframeMapper::publishKeyframesSrvCallback(
  PublishKeyframes::Request& request,
  PublishKeyframes::Response& response)
{
  // regex matching - try match the request string against each
  // keyframe index
  
  bool found_match = false;

  boost::regex expression(request.re);
  
  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    std::stringstream ss;
    ss << kf_idx;
    std::string kf_idx_string = ss.str();
      
    boost::smatch match;
    
    if(boost::regex_match(kf_idx_string, match, expression))
    {
      found_match = true;
      ROS_INFO("Publishing keyframe %d", kf_idx);
      publishKeyframeData(kf_idx);
      publishKeyframePose(kf_idx);
      usleep(25000);
    }
  }

  return found_match;
}

void KeyframeMapper::publishKeyframeData(int i)
{
  RGBDKeyframe& keyframe = keyframes_[i];

  // data transformed to the fixed frame
  PointCloudT keyframe_data_ff; 
  pcl::transformPointCloud(
    keyframe.cloud, keyframe_data_ff, eigenFromTf(keyframe.pose));

  keyframe_data_ff.header.frame_id = fixed_frame_;

  keyframes_pub_.publish(keyframe_data_ff);
}

void KeyframeMapper::publishKeyframeAssociations()
{
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = fixed_frame_;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;

  marker.points.resize(associations_.size() * 2);
  
  marker.color.a = 1.0;

  for (unsigned int as_idx = 0; as_idx < associations_.size(); ++as_idx)
  {
    // set up shortcut references
    const KeyframeAssociation& association = associations_[as_idx];
    int kf_idx_a = association.kf_idx_a;
    int kf_idx_b = association.kf_idx_b;
    RGBDKeyframe& keyframe_a = keyframes_[kf_idx_a];
    RGBDKeyframe& keyframe_b = keyframes_[kf_idx_b];

    int idx_start = as_idx*2;
    int idx_end   = as_idx*2 + 1;

    // start point for the edge
    marker.points[idx_start].x = keyframe_a.pose.getOrigin().getX();  
    marker.points[idx_start].y = keyframe_a.pose.getOrigin().getY();
    marker.points[idx_start].z = keyframe_a.pose.getOrigin().getZ();

    // end point for the edge
    marker.points[idx_end].x = keyframe_b.pose.getOrigin().getX();  
    marker.points[idx_end].y = keyframe_b.pose.getOrigin().getY();
    marker.points[idx_end].z = keyframe_b.pose.getOrigin().getZ();

    if (association.type == KeyframeAssociation::VO)
    {
      marker.ns = "VO";
      marker.scale.x = 0.002;

      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    else if (association.type == KeyframeAssociation::RANSAC)
    {
      marker.ns = "RANSAC";
      marker.scale.x = 0.002;
      
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }

    kf_assoc_pub_.publish(marker);
  }
}

void KeyframeMapper::publishKeyframePoses()
{
  for(unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    publishKeyframePose(kf_idx);
  }
}

void KeyframeMapper::publishKeyframePose(int i)
{
  RGBDKeyframe& keyframe = keyframes_[i];

  // **** publish camera pose

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = fixed_frame_;
  marker.ns = "keyframe_poses";
  marker.id = i;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.points.resize(2);

  // start point for the arrow
  marker.points[0].x = keyframe.pose.getOrigin().getX();
  marker.points[0].y = keyframe.pose.getOrigin().getY();
  marker.points[0].z = keyframe.pose.getOrigin().getZ();

  // end point for the arrow
  tf::Transform ep; 
  ep.setIdentity();
  ep.setOrigin(tf::Vector3(0.00, 0.00, 0.12)); // z = arrow length
  ep = keyframe.pose * ep;

  marker.points[1].x = ep.getOrigin().getX();
  marker.points[1].y = ep.getOrigin().getY();
  marker.points[1].z = ep.getOrigin().getZ(); 
  
  marker.scale.x = 0.02; // shaft radius
  marker.scale.y = 0.05; // head radius

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  poses_pub_.publish(marker);

  // **** publish frame index text

  visualization_msgs::Marker marker_text;
  marker_text.header.stamp = ros::Time::now();
  marker_text.header.frame_id = fixed_frame_;
  marker_text.ns = "keyframe_indexes";
  marker_text.id = i;
  marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_text.action = visualization_msgs::Marker::ADD;

  tf::poseTFToMsg(keyframe.pose, marker_text.pose);

  marker_text.pose.position.z -= 0.05;

  char label[6];
  sprintf(label, "%d", i);
  marker_text.text = label;

  marker_text.color.a = 1.0;
  marker_text.color.r = 1.0;
  marker_text.color.g = 1.0;
  marker_text.color.b = 0.0;

  marker_text.scale.z = 0.05; // shaft radius

  poses_pub_.publish(marker_text);
}

bool KeyframeMapper::saveKeyframesSrvCallback(
  Save::Request& request,
  Save::Response& response)
{
  ROS_INFO("Saving keyframes...");
  std::string path = request.filename;
  return saveKeyframes(keyframes_, path);
}

bool KeyframeMapper::loadKeyframesSrvCallback(
  Load::Request& request,
  Load::Response& response)
{
  ROS_INFO("Loading keyframes...");
  std::string path = request.filename;
  return loadKeyframes(keyframes_, path);
}

bool KeyframeMapper::saveFullSrvCallback(
  Save::Request& request,
  Save::Response& response)
{
  ROS_INFO("Saving full map...");
  std::string path = request.filename;
  return saveFullMap(path);
}

bool KeyframeMapper::saveFullMap(const std::string& path)
{
  double full_map_res_ = 0.01;

  PointCloudT::Ptr full_map(new PointCloudT());
  full_map->header.frame_id = fixed_frame_;

  // aggregate all frames into single cloud
  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    const RGBDKeyframe& keyframe = keyframes_[kf_idx];

    PointCloudT cloud_tf;
    pcl::transformPointCloud(keyframe.cloud, cloud_tf, eigenFromTf(keyframe.pose));
    cloud_tf.header.frame_id = fixed_frame_;

    *full_map += cloud_tf;
  }

  // filter cloud
  PointCloudT full_map_f;
  pcl::VoxelGrid<PointT> vgf;
  vgf.setInputCloud(full_map);
  vgf.setLeafSize(full_map_res_, full_map_res_, full_map_res_);
  vgf.filter(full_map_f);

  // write out
  pcl::PCDWriter writer;
  int result_pcd = writer.writeBinary<PointT>(path + ".pcd", full_map_f);  

  return result_pcd;
}

bool KeyframeMapper::addManualKeyframeSrvCallback(
  AddManualKeyframe::Request& request,
  AddManualKeyframe::Response& response)
{
  manual_add_ = true;

  return true;
}

bool KeyframeMapper::generateGraphSrvCallback(
  GenerateGraph::Request& request,
  GenerateGraph::Response& response)
{
  associations_.clear();
  graph_detector_.generateKeyframeAssociations(keyframes_, associations_);

  publishKeyframePoses();
  publishKeyframeAssociations();

  return true;
}

bool KeyframeMapper::solveGraphSrvCallback(
  SolveGraph::Request& request,
  SolveGraph::Response& response)
{
  graph_solver_->solve(keyframes_, associations_);

  publishKeyframePoses();
  publishKeyframeAssociations();

  return true;
}

bool KeyframeMapper::saveOctomapSrvCallback(
  Save::Request& request,
  Save::Response& response)
{
  saveOctomap(request.filename);
  return true;
}

void KeyframeMapper::saveOctomap(const std::string& path)
{ 
  ROS_INFO("Building octomap...");
  //octomap::OcTree tree(octomap_res_);   
  //buildOctomap(tree);
  
  octomap::ColorOcTree tree(octomap_res_);   
  buildColorOctomap(tree);
  
  ROS_INFO("Saving octomap...");
  //std::ofstream stream(path.c_str());
  //tree.wite(stream);
  
  tree.write(path);
  
  //tree.writeColorHistogram(path + "hist");
  
  ROS_INFO("Done");
}

void KeyframeMapper::buildOctomap(octomap::OcTree& tree)
{
  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    ROS_INFO("Processing keyframe %u", kf_idx);
    const RGBDKeyframe& keyframe = keyframes_[kf_idx];
    const PointCloudT& cloud = keyframe.cloud;

    // build octomap cloud from pcl cloud
    octomap::Pointcloud octomap_cloud;
    for (unsigned int pt_idx = 0; pt_idx < cloud.points.size(); ++pt_idx)
    {
      const PointT& p = cloud.points[pt_idx];
      if (!std::isnan(p.z))
        octomap_cloud.push_back(p.x, p.y, p.z);
    }
    
    octomap::point3d sensor_origin(0.0, 0.0, 0.0);  
    octomap::pose6d frame_origin = poseTfToOctomap(keyframe.pose);
    
    tree.insertScan(octomap_cloud, sensor_origin, frame_origin);
  }
}

void KeyframeMapper::buildColorOctomap(octomap::ColorOcTree& tree)
{/*
  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    ROS_INFO("Processing keyframe %u", kf_idx);
    const RGBDKeyframe& keyframe = keyframes_[kf_idx];
    const PointCloudT& cloud = keyframe.cloud;

    octomap::point3d sensor_origin(0.0, 0.0, 0.0);  
    octomap::pose6d frame_origin = poseTfToOctomap(keyframe.pose);
    
    // insert into map one by one
    octomap::Pointcloud octomap_cloud;
    for (unsigned int pt_idx = 0; pt_idx < cloud.points.size(); ++pt_idx)
    {
      const PointT& p = cloud.points[pt_idx];
      if (!std::isnan(p.z))
      {
        octomap::point3d endpoint(p.x, p.y, p.z);
        octomap::ColorOcTreeNode* n = tree.updateNode(endpoint, true); 
        n->setColor(p.r, p.g, p.b); // set color to yellow
      }
    }
    
    tree.updateInnerOccupancy();
  }
  */
  
  
    for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    ROS_INFO("Processing keyframe %u", kf_idx);
    const RGBDKeyframe& keyframe = keyframes_[kf_idx];
    const PointCloudT& cloud = keyframe.cloud;

    // build octomap cloud from pcl cloud
    octomap::Pointcloud octomap_cloud;
    for (unsigned int pt_idx = 0; pt_idx < cloud.points.size(); ++pt_idx)
    {
      const PointT& p = cloud.points[pt_idx];
      if (!std::isnan(p.z))
        octomap_cloud.push_back(p.x, p.y, p.z);
    }
    
    octomap::point3d sensor_origin(0.0, 0.0, 0.0);  
    octomap::pose6d frame_origin = poseTfToOctomap(keyframe.pose);
    
    // insert scan
    tree.insertScan(octomap_cloud, sensor_origin, frame_origin);
    
    // insert colors
    PointCloudT cloud_tf;
    pcl::transformPointCloud(cloud, cloud_tf, eigenFromTf(keyframe.pose));
    for (unsigned int pt_idx = 0; pt_idx < cloud_tf.points.size(); ++pt_idx)
    {
      const PointT& p = cloud_tf.points[pt_idx];
      if (!std::isnan(p.z))
      {
        octomap::point3d endpoint(p.x, p.y, p.z);
        octomap::ColorOcTreeNode* n = tree.search(endpoint);
        if (n) n->setColor(p.r, p.g, p.b); 
      }
    }
    
    tree.updateInnerOccupancy();
  }
}

} // namespace ccny_rgbd
