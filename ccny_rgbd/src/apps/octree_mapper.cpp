#include "ccny_rgbd/apps/octree_mapper.h"

namespace ccny_rgbd {

OctreeMapper::OctreeMapper(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
  ROS_INFO("Starting Octree Mapper");

  initParams();

  map_ = boost::make_shared<PointCloudT>();
  map_->header.frame_id = fixed_frame_;

  octree_ = new Octree(resolution_);
  octree_->setInputCloud(map_);

  // **** publishers

  map_pub_ = nh_.advertise<PointCloudT>(
   "map", 1);
  marker_pub_ = nh_.advertise<MarkerArrayMsg>(
   "map_markers", 1);

  // **** services

  save_service_ = nh_.advertiseService(
    "save_map", &OctreeMapper::saveSrvCallback, this);

  // **** subscribers

  image_transport::ImageTransport rgb_it(nh_);
  image_transport::ImageTransport depth_it(nh_);

  sub_depth_.subscribe(
    depth_it, "/camera/depth_registered/image_rect_raw", 1);
  sub_rgb_.subscribe(
    rgb_it, "/camera/rgb/image_rect_color", 1);
  sub_info_.subscribe(
    nh_, "/camera/rgb/camera_info", 1);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  int queue_size = 5;
  sync_.reset(new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_));
  sync_->registerCallback(boost::bind(&OctreeMapper::RGBDCallback, this, _1, _2, _3));  

}

OctreeMapper::~OctreeMapper()
{
  ROS_INFO("Destroying Octree Mapper");
}

void OctreeMapper::initParams()
{
  if (!nh_private_.getParam ("resolution", resolution_))
    resolution_ = 0.01;
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "odom";
  if (!nh_private_.getParam ("max_range", max_range_))
    max_range_ = 4.0;
}

void OctreeMapper::RGBDCallback(
  const ImageMsg::ConstPtr& depth_msg,
  const ImageMsg::ConstPtr& rgb_msg,
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
    
  // TODO: cleanup constructors
  RGBDFrame frame(rgb_msg, depth_msg, info_msg);
  RGBDKeyframe keyframe(frame);
  keyframe.constructDataCloud();

  // transform to world frame
  PointCloudT cloud_tf;
  pcl_ros::transformPointCloud (keyframe.data, cloud_tf, transform);
  cloud_tf.header.frame_id = fixed_frame_;

  // add to map
  boost::mutex::scoped_lock(mutex_);

  for (unsigned int i = 0; i < cloud_tf.points.size(); ++i)
  {
    int index;
    const PointT& p = cloud_tf.points[i];
    if (isnan(p.z)) continue; 
    octree_->addPointWithReplacement(p, map_, index);
  }
  map_->header.stamp = cloud_tf.header.stamp;

  publishPointMap();
  //publishMarkerMap();
}

void OctreeMapper::publishPointMap()
{
  map_pub_.publish(map_);
}

void OctreeMapper::publishMarkerMap()
{
  MarkerMsg marker_template;
  MarkerArrayMsg marker_array;

  marker_array.markers.resize(map_->points.size());

  marker_template.header.frame_id = fixed_frame_;
  marker_template.header.stamp = ros::Time::now(); // TODO
  marker_template.ns = "map";
  marker_template.type = visualization_msgs::Marker::CUBE;
  marker_template.action = visualization_msgs::Marker::ADD;

  marker_template.scale.x = resolution_;
  marker_template.scale.y = resolution_;
  marker_template.scale.z = resolution_;

  marker_template.pose.orientation.x = 0.0;
  marker_template.pose.orientation.y = 0.0;
  marker_template.pose.orientation.z = 0.0;
  marker_template.pose.orientation.w = 1.0;

  for (unsigned int i = 0; i < map_->points.size(); ++i)
  {
    const PointT& p = map_->points[i];

    MarkerMsg marker(marker_template);
    marker.id = i;

    pcl::octree::OctreePointCloudSearch<PointT, LeafT, OctreeT>::AlignedPointTVector voxel_list;

    octree_->getApproxIntersectedVoxelCentersBySegment(
      Eigen::Vector3f(p.x, p.y, p.z), 
      Eigen::Vector3f(p.x, p.y, p.z+1.0), 
      voxel_list);

    if (voxel_list.empty()) continue;

    marker.pose.position.x = voxel_list[0].x;
    marker.pose.position.y = voxel_list[0].y;
    marker.pose.position.z = voxel_list[0].z;

    //marker.pose.position.x = p.x;
    //marker.pose.position.y = p.y;
    //marker.pose.position.z = p.z;

    marker.color.a = 1.0;
    marker.color.r = p.r / 255.0;
    marker.color.g = p.g / 255.0;
    marker.color.b = p.b / 255.0;

    marker_array.markers[i] = marker;
  }

  marker_pub_.publish(marker_array);
}

bool OctreeMapper::saveSrvCallback(
  ccny_rgbd::Save::Request& request,
  ccny_rgbd::Save::Response& response)
{
  ROS_INFO("Saving to %s...", request.filename.c_str());
  ROS_INFO("Map has %dK points", (int)map_->points.size()/1024);

  pcl::io::savePCDFileBinary<PointT>(request.filename, *map_);
  //pcl::io::savePLYFile<PointT>(request.filename + ".ply", *map_);

  ROS_INFO("Save successful");
  return true;
}

} // namespace ccny_rgbd
