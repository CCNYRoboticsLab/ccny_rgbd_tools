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
  rgbd_frame_index_(0)
{
  ROS_INFO("Starting RGBD Keyframe Mapper");
   
  // **** params
  
  initParams();
  
  // **** publishers
  
  keyframes_pub_ = nh_.advertise<PointCloudT>(
    "keyframes", queue_size_);
  poses_pub_ = nh_.advertise<visualization_msgs::Marker>( 
    "keyframe_poses", queue_size_);
  kf_assoc_pub_ = nh_.advertise<visualization_msgs::Marker>( 
    "keyframe_associations", queue_size_);
  path_pub_ = nh_.advertise<PathMsg>( 
    "mapper_path", queue_size_);
  
  // **** services
  
  pub_keyframe_service_ = nh_.advertiseService(
    "publish_keyframe", &KeyframeMapper::publishKeyframeSrvCallback, this);
  pub_keyframes_service_ = nh_.advertiseService(
    "publish_keyframes", &KeyframeMapper::publishKeyframesSrvCallback, this);
  save_kf_service_ = nh_.advertiseService(
    "save_keyframes", &KeyframeMapper::saveKeyframesSrvCallback, this);
  load_kf_service_ = nh_.advertiseService(
    "load_keyframes", &KeyframeMapper::loadKeyframesSrvCallback, this);   
  save_pcd_map_service_ = nh_.advertiseService(
    "save_pcd_map", &KeyframeMapper::savePcdMapSrvCallback, this);
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

}

void KeyframeMapper::initParams()
{
  if (!nh_private_.getParam ("queue_size", queue_size_))
    queue_size_ = 5;
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("pcd_map_res", pcd_map_res_))
    pcd_map_res_ = 0.01;
  if (!nh_private_.getParam ("octomap_res", octomap_res_))
    octomap_res_ = 0.05;
  if (!nh_private_.getParam ("octomap_with_color", octomap_with_color_))
   octomap_with_color_ = true;
  if (!nh_private_.getParam ("kf_dist_eps", kf_dist_eps_))
    kf_dist_eps_  = 0.10;
  if (!nh_private_.getParam ("kf_angle_eps", kf_angle_eps_))
    kf_angle_eps_  = 10.0 * M_PI / 180.0;
  if (!nh_private_.getParam ("max_range", max_range_))
    max_range_  = 5.5;
  if (!nh_private_.getParam ("max_stdev", max_stdev_))
    max_stdev_  = 0.03;
  if (!nh_private_.getParam ("max_map_z", max_map_z_))
    max_map_z_ = std::numeric_limits<double>::infinity();
   
  // configure graph detection 
    
  int max_ransac_iterations;  
  int n_ransac_candidates;
  int k_nearest_neighbors;
  int min_ransac_inliers;
  int n_keypoints;
  double max_corresp_dist_desc;
  double max_corresp_dist_eucl;
    
  if (!nh_private_.getParam ("graph/max_ransac_iterations", max_ransac_iterations))
    max_ransac_iterations = 2000;
  if (!nh_private_.getParam ("graph/n_ransac_candidates", n_ransac_candidates))
    n_ransac_candidates = 15;
  if (!nh_private_.getParam ("graph/k_nearest_neighbors", k_nearest_neighbors))
    k_nearest_neighbors = 15;
  if (!nh_private_.getParam ("graph/min_ransac_inliers", min_ransac_inliers))
    min_ransac_inliers = 30;
  if (!nh_private_.getParam ("graph/max_corresp_dist_desc", max_corresp_dist_desc))
    max_corresp_dist_desc = 1.0;
  if (!nh_private_.getParam ("graph/max_corresp_dist_eucl", max_corresp_dist_eucl))
    max_corresp_dist_eucl = 0.03;
  if (!nh_private_.getParam ("graph/n_keypoints", n_keypoints))
    n_keypoints = 200;
    
  graph_detector_.setMaxRansacIterations(max_ransac_iterations);    
  graph_detector_.setNRansacCandidates(n_ransac_candidates);   
  graph_detector_.setKNearestNeighbors(k_nearest_neighbors); 
  graph_detector_.setMinRansacInliers(min_ransac_inliers);
  graph_detector_.setNKeypoints(n_keypoints);
  graph_detector_.setMaxCorrespDistDesc(max_corresp_dist_desc);
  graph_detector_.setMaxCorrespDistEucl(max_corresp_dist_eucl);
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
     fixed_frame_, rgb_msg->header.frame_id, time, ros::Duration(0.33));
    tf_listener_.lookupTransform(
      fixed_frame_, rgb_msg->header.frame_id, time, transform);  
  }
  catch(tf::TransformException& ex)
  {
    ROS_WARN("Could not receive transform. Skipping image: %s", ex.what());
    return;
  }
  
  // create a new frame and increment the counter
  rgbdtools::RGBDFrame frame;
  createRGBDFrameFromROSMessages(rgb_msg, depth_msg, info_msg, frame); 
  frame.index = rgbd_frame_index_;
  rgbd_frame_index_++;
  
  bool result = processFrame(frame, eigenAffineFromTf(transform));
  if (result) publishKeyframeData(keyframes_.size() - 1);
  
  publishPath();
}



bool KeyframeMapper::processFrame(
  const rgbdtools::RGBDFrame& frame, 
  const AffineTransform& pose)
{
  // add the frame pose to the path vector
  geometry_msgs::PoseStamped frame_pose; 
  tf::Transform frame_tf = tfFromEigenAffine(pose);
  tf::poseTFToMsg(frame_tf, frame_pose.pose);
 
  // update the header of the pose for the path
  frame_pose.header.frame_id = fixed_frame_;
  frame_pose.header.seq = frame.header.seq;
  frame_pose.header.stamp.sec = frame.header.stamp.sec;
  frame_pose.header.stamp.nsec = frame.header.stamp.nsec;
    
  path_msg_.poses.push_back(frame_pose);
   
  // determine if a new keyframe is needed
  bool result; 

  if(keyframes_.empty() || manual_add_)
  {
    result = true;
  }
  else
  {
    double dist, angle;
    getTfDifference(tfFromEigenAffine(pose), 
                    tfFromEigenAffine(keyframes_.back().pose), 
                    dist, angle);

    if (dist > kf_dist_eps_ || angle > kf_angle_eps_)
      result = true;
    else 
      result = false;
  }

  if (result)
  {
    addKeyframe(frame, pose);
  }
  return result;
}

void KeyframeMapper::addKeyframe(
  const rgbdtools::RGBDFrame& frame, 
  const AffineTransform& pose)
{
  rgbdtools::RGBDKeyframe keyframe(frame);
  keyframe.pose = pose;
  
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
  bool found_match = false;

  // regex matching - try match the request string against each
  // keyframe index
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

  publishPath();

  return found_match;
}

void KeyframeMapper::publishKeyframeData(int i)
{
  rgbdtools::RGBDKeyframe& keyframe = keyframes_[i];

  // construct a cloud from the images
  PointCloudT cloud;
  keyframe.constructDensePointCloud(cloud, max_range_, max_stdev_);
  
  // cloud transformed to the fixed frame
  PointCloudT cloud_ff; 
  pcl::transformPointCloud(cloud, cloud_ff, keyframe.pose);

  cloud_ff.header.frame_id = fixed_frame_;

  keyframes_pub_.publish(cloud_ff);
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
    const rgbdtools::KeyframeAssociation& association = associations_[as_idx];
    int kf_idx_a = association.kf_idx_a;
    int kf_idx_b = association.kf_idx_b;
    rgbdtools::RGBDKeyframe& keyframe_a = keyframes_[kf_idx_a];
    rgbdtools::RGBDKeyframe& keyframe_b = keyframes_[kf_idx_b];

    int idx_start = as_idx*2;
    int idx_end   = as_idx*2 + 1;

    tf::Transform keyframe_a_pose = tfFromEigenAffine(keyframe_a.pose);
    tf::Transform keyframe_b_pose = tfFromEigenAffine(keyframe_b.pose);
 
    // start point for the edge
    marker.points[idx_start].x = keyframe_a_pose.getOrigin().getX();  
    marker.points[idx_start].y = keyframe_a_pose.getOrigin().getY();
    marker.points[idx_start].z = keyframe_a_pose.getOrigin().getZ();

    // end point for the edge
    marker.points[idx_end].x = keyframe_b_pose.getOrigin().getX();  
    marker.points[idx_end].y = keyframe_b_pose.getOrigin().getY();
    marker.points[idx_end].z = keyframe_b_pose.getOrigin().getZ();

    if (association.type == rgbdtools::KeyframeAssociation::VO)
    {
      marker.ns = "VO";
      marker.scale.x = 0.002;

      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    else if (association.type == rgbdtools::KeyframeAssociation::RANSAC)
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
  rgbdtools::RGBDKeyframe& keyframe = keyframes_[i];

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
  tf::Transform keyframe_pose = tfFromEigenAffine(keyframe.pose);
  marker.points[0].x = keyframe_pose.getOrigin().getX();
  marker.points[0].y = keyframe_pose.getOrigin().getY();
  marker.points[0].z = keyframe_pose.getOrigin().getZ();

  // end point for the arrow
  tf::Transform ep; 
  ep.setIdentity();
  ep.setOrigin(tf::Vector3(0.00, 0.00, 0.12)); // z = arrow length
  ep = keyframe_pose * ep;

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

  tf::poseTFToMsg(keyframe_pose, marker_text.pose);

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
  std::string filepath = request.filename;

  ROS_INFO("Saving keyframes...");
  bool result_kf = saveKeyframes(keyframes_, filepath);
  if (result_kf) ROS_INFO("Keyframes saved to %s", filepath.c_str());
  else ROS_ERROR("Keyframe saving failed!");
  
  ROS_INFO("Saving path...");
  bool result_path = savePath(filepath);
  if (result_path ) ROS_INFO("Path saved to %s", filepath.c_str());
  else ROS_ERROR("Path saving failed!");
    
  return result_kf && result_path;
}

bool KeyframeMapper::loadKeyframesSrvCallback(
  Load::Request& request,
  Load::Response& response)
{
  std::string filepath = request.filename;
  
  ROS_INFO("Loading keyframes...");
  bool result_kf = loadKeyframes(keyframes_, filepath); 
  if (result_kf) ROS_INFO("Keyframes loaded successfully");
  else ROS_ERROR("Keyframe loading failed!");
  
  ROS_INFO("Loading path...");
  bool result_path = loadPath(filepath);
  if (result_path) ROS_INFO("Path loaded successfully");
  else ROS_ERROR("Path loading failed!");
  
  return result_kf && result_path;
}

bool KeyframeMapper::savePcdMapSrvCallback(
  Save::Request& request,
  Save::Response& response)
{
  ROS_INFO("Saving map as pcd...");
  const std::string& path = request.filename; 
  bool result = savePcdMap(path);
  
  if (result) ROS_INFO("Pcd map saved to %s", path.c_str());
  else ROS_ERROR("Pcd map saving failed");
  
  return result;
}

bool KeyframeMapper::saveOctomapSrvCallback(
  Save::Request& request,
  Save::Response& response)
{
  ROS_INFO("Saving map as Octomap...");
  const std::string& path = request.filename;
  bool result = saveOctomap(path);
    
  if (result) ROS_INFO("Octomap saved to %s", path.c_str());
  else ROS_ERROR("Octomap saving failed");
    
  return result;
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
  ros::WallTime start = ros::WallTime::now();
  graph_solver_.solve(keyframes_, associations_);
  //graph_solver_.solve(keyframe_path_indices_, path_, associations_);
  double dur = getMsDuration(start);
  
  ROS_INFO("Solving took %.1f ms", dur);
  
  //updatePathFromKeyframePoses();
  
  publishPath();
  publishKeyframePoses();
  publishKeyframeAssociations();

  return true;
}


/** In the event that the keyframe poses change (from pose-graph solving)
 * this function will propagete teh changes in the path message
 */

/*
void KeyframeMapper::updatePathFromKeyframePoses()
{
  int kf_size = keyframes_.size();
  int f_size = path_msg_.poses.size();
  
  if (kf_size < 2) return;
  
  // the frame index
  int f_idx = 0;
  
  for (int kf_idx = 0; kf_idx < kf_size - 1; ++kf_idx)
  {
    int kf_idx_next = kf_idx +1;
    
    const rgbdtools::RGBDKeyframe& keyframe = keyframes_[kf_idx];
    const rgbdtools::RGBDKeyframe& keyframe_next = keyframes_[kf_idx_next];
    
    int seq      = keyframe.header.seq;
    int seq_next = keyframe_next.header.seq;
    
    // find f_idx_next
    int f_idx_next = f_idx;
    
    while(true)
    {
      assert(f_idx_next < f_size);
      if (path_msg_.poses[f_idx_next].header.seq == seq_next) break;
      f_idx_next++;
    }
    
    tf::Transform pa = tfFromEigenAffine(keyframe.pose);
    tf::Transform pb = tfFromEigenAffine(keyframe_next.pose);
    tf::Transform pb_old;
    tf::poseMsgToTF(path_msg_.poses[f_idx_next].pose, pb_old);
    
    tf_transform delta_new = pa.inverse() * pb;
    tf_transform delta_old = pa.inverse() * pb_old;
    
    tf_transform D = delta_old.inverse() * delta_new();
    
    
    
  }
  
}
*/

bool KeyframeMapper::savePcdMap(const std::string& path)
{
  PointCloudT pcd_map;
  buildPcdMap(pcd_map);
  
  // write out
  pcl::PCDWriter writer;
  int result_pcd = writer.writeBinary<PointT>(path, pcd_map);  

  if (result_pcd < 0) return false;
  else return true;
}

void KeyframeMapper::buildPcdMap(PointCloudT& map_cloud)
{
  PointCloudT::Ptr aggregate_cloud(new PointCloudT());
  aggregate_cloud->header.frame_id = fixed_frame_;

  // aggregate all frames into single cloud
  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    const rgbdtools::RGBDKeyframe& keyframe = keyframes_[kf_idx];
    
    PointCloudT cloud;   
    keyframe.constructDensePointCloud(cloud, max_range_, max_stdev_);

    PointCloudT cloud_tf;
    pcl::transformPointCloud(cloud, cloud_tf, keyframe.pose);
    cloud_tf.header.frame_id = fixed_frame_;

    *aggregate_cloud += cloud_tf;
  }

  // filter cloud using voxel grid, and for max z
  pcl::VoxelGrid<PointT> vgf;
  vgf.setInputCloud(aggregate_cloud);
  vgf.setLeafSize(pcd_map_res_, pcd_map_res_, pcd_map_res_);
  vgf.setFilterFieldName("z");
  vgf.setFilterLimits (-std::numeric_limits<double>::infinity(), max_map_z_);

  vgf.filter(map_cloud);
}

bool KeyframeMapper::saveOctomap(const std::string& path)
{
  bool result;

  if (octomap_with_color_)
  {
    octomap::ColorOcTree tree(octomap_res_);   
    buildColorOctomap(tree);
    result = tree.write(path);
  }
  else
  {
    octomap::OcTree tree(octomap_res_);   
    buildOctomap(tree);
    result = tree.write(path);
  }
  
  return result;
}

void KeyframeMapper::buildOctomap(octomap::OcTree& tree)
{
  ROS_INFO("Building Octomap...");
  
  octomap::point3d sensor_origin(0.0, 0.0, 0.0);  

  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    ROS_INFO("Processing keyframe %u", kf_idx);
    const rgbdtools::RGBDKeyframe& keyframe = keyframes_[kf_idx];
    
    PointCloudT cloud;
    keyframe.constructDensePointCloud(cloud, max_range_, max_stdev_);
           
    octomap::pose6d frame_origin = poseTfToOctomap(tfFromEigenAffine(keyframe.pose));

    // build octomap cloud from pcl cloud
    octomap::Pointcloud octomap_cloud;
    for (unsigned int pt_idx = 0; pt_idx < cloud.points.size(); ++pt_idx)
    {
      const PointT& p = cloud.points[pt_idx];
      if (!std::isnan(p.z))
        octomap_cloud.push_back(p.x, p.y, p.z);
    }
    
    tree.insertScan(octomap_cloud, sensor_origin, frame_origin);
  }
}

void KeyframeMapper::buildColorOctomap(octomap::ColorOcTree& tree)
{
  ROS_INFO("Building Octomap with color...");

  octomap::point3d sensor_origin(0.0, 0.0, 0.0);  

  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    ROS_INFO("Processing keyframe %u", kf_idx);
    const rgbdtools::RGBDKeyframe& keyframe = keyframes_[kf_idx];
       
    // construct the cloud
    PointCloudT::Ptr cloud_unf(new PointCloudT());
    keyframe.constructDensePointCloud(*cloud_unf, max_range_, max_stdev_);
  
    // perform filtering for max z
    pcl::transformPointCloud(*cloud_unf, *cloud_unf, keyframe.pose);
    PointCloudT cloud;
    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloud_unf);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-std::numeric_limits<double>::infinity(), max_map_z_);
    pass.filter(cloud);
    pcl::transformPointCloud(cloud, cloud, keyframe.pose.inverse());
    
    octomap::pose6d frame_origin = poseTfToOctomap(tfFromEigenAffine(keyframe.pose));
    
    // build octomap cloud from pcl cloud
    octomap::Pointcloud octomap_cloud;
    for (unsigned int pt_idx = 0; pt_idx < cloud.points.size(); ++pt_idx)
    {
      const PointT& p = cloud.points[pt_idx];
      if (!std::isnan(p.z))
        octomap_cloud.push_back(p.x, p.y, p.z);
    }
    
    // insert scan (only xyz considered, no colors)
    tree.insertScan(octomap_cloud, sensor_origin, frame_origin);
    
    // insert colors
    PointCloudT cloud_tf;
    pcl::transformPointCloud(cloud, cloud_tf, keyframe.pose);
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

void KeyframeMapper::publishPath()
{
  path_msg_.header.frame_id = fixed_frame_; 
  path_pub_.publish(path_msg_);
}

bool KeyframeMapper::savePath(const std::string& filepath)
{
  // open file
  std::string filename = filepath + "/path.txt";
  std::ofstream file(filename.c_str());
  if (!file.is_open()) return false;

  file << "# index seq stamp.sec stamp.nsec x y z qx qy qz qw" << std::endl;

  for (unsigned int idx = 0; idx < path_msg_.poses.size(); ++idx)
  {
    const geometry_msgs::PoseStamped& pose = path_msg_.poses[idx];
    
    file << idx << " "
         << pose.header.seq << " "
         << pose.header.stamp.sec << " "
         << pose.header.stamp.nsec << " "
         << pose.pose.position.x << " "
         << pose.pose.position.y << " "
         << pose.pose.position.z << " "
         << pose.pose.orientation.x << " "
         << pose.pose.orientation.y << " "
         << pose.pose.orientation.z << " " 
         << pose.pose.orientation.w << std::endl;
  }

  file.close();
  
  return true;
}

bool KeyframeMapper::loadPath(const std::string& filepath)
{
  path_msg_.poses.clear();

  // open file
  std::string filename = filepath + "/path.txt";
  std::ifstream file(filename.c_str());
  if (!file.is_open()) return false;
  
  std::string line;

  // get header
  getline(file, line);
  std::cout << line << std::endl;

  // read each line
  while(std::getline(file, line))
  {
    std::istringstream is(line);
    //std::cout << "[" << is.str() << "]" << std::endl;
    
    // fill out pose information  
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = fixed_frame_;
    int idx;
       
    is >> idx
       >> pose.header.seq 
       >> pose.header.stamp.sec 
       >> pose.header.stamp.nsec 
       >> pose.pose.position.x 
       >> pose.pose.position.y 
       >> pose.pose.position.z 
       >> pose.pose.orientation.x 
       >> pose.pose.orientation.y 
       >> pose.pose.orientation.z 
       >> pose.pose.orientation.w;
                 
    // add to poses vector  
    path_msg_.poses.push_back(pose);
  }
    
  file.close();
  return true;
}

} // namespace ccny_rgbd
