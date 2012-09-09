#include "ccny_3dmap_server/map_server.h"

namespace ccny_rgbd {

MapServer::MapServer(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh),
  nh_private_(nh_private),
  resolution_(0.01),
  max_range_(4.0)
{
  ROS_INFO("Starting Map Server");

  initParams();

  map_ = boost::make_shared<PointCloudT>();
  map_->header.frame_id = fixed_frame_;

  octree_ = new Octree(resolution_);
  octree_->setInputCloud(map_);

  // **** publishers

  map_pub_ = nh_.advertise<PointCloudT>(
   map_topic_, 1);

  // **** subscribers

  point_cloud_subscriber_ = nh_.subscribe<PointCloudT>(
    sub_topic_, 1, &MapServer::pointCloudCallback, this);

  // **** services

  save_service_ = nh_.advertiseService(
    "save_map", &MapServer::saveSrvCallback, this);
}

MapServer::~MapServer()
{
  ROS_INFO("Destroying Map Server");

}

void MapServer::initParams()
{

  if (!nh_private_.getParam ("resolution", resolution_))
    resolution_ = 0.01;
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "world";
  if (!nh_private_.getParam ("max_range", max_range_))
    max_range_ = 4.0;

}

void MapServer::pointCloudCallback(const PointCloudT::ConstPtr& cloud_in_ptr)
{
  struct timeval start, end;
  gettimeofday(&start, NULL);

  // **** get fixed frame to camera tf ************************************************

  tf::StampedTransform f2c_tf;

  try
  {
    tf_listener_.waitForTransform (
      fixed_frame_, cloud_in_ptr->header.frame_id, cloud_in_ptr->header.stamp, ros::Duration(1.0));
    tf_listener_.lookupTransform (
      fixed_frame_, cloud_in_ptr->header.frame_id, cloud_in_ptr->header.stamp, f2c_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("TF unavailable %s", ex.what());
    return;
  }

  tf::Transform f2c = f2c_tf;

  // **** create data cloud  ***************************************************

  PointCloudT::Ptr data_ptr = boost::make_shared<PointCloudT>();
  filterCloud(cloud_in_ptr, data_ptr);

  // **** transform to best guess for world frame ******************************

  pcl_ros::transformPointCloud (*data_ptr, *data_ptr, f2c);
  data_ptr->header.frame_id = fixed_frame_;

  // **** add to map *********************************************************

  boost::mutex::scoped_lock(mutex_);

  for (unsigned int i = 0; i < data_ptr->points.size(); ++i)
  {
    int index;
    bool replaced = octree_->addPointWithReplacement(data_ptr->points[i], map_, index);
  }
  map_->header.stamp = cloud_in_ptr->header.stamp;

  publishMap();

  gettimeofday(&end, NULL);
}

void MapServer::publishMap()
{
  boost::mutex::scoped_lock(mutex_);
  map_pub_.publish(map_);
}

void MapServer::filterCloud(const PointCloudT::ConstPtr& cloud_in_ptr,
                            const PointCloudT::Ptr& data_ptr)
{
  for (unsigned int i = 0; i < cloud_in_ptr->points.size(); ++i)
  {
    float z = cloud_in_ptr->points[i].z;

    if (!isnan(z) && z < max_range_)
      data_ptr->points.push_back(cloud_in_ptr->points[i]);
  }

  data_ptr->is_dense = true;
  data_ptr->width  = data_ptr->points.size();
  data_ptr->height = 1;
  data_ptr->header = cloud_in_ptr->header;
}

bool MapServer::saveSrvCallback(
  ccny_3dmap_server::Save::Request& request,
  ccny_3dmap_server::Save::Response& response)
{
  ROS_INFO("Saving to %s...", request.filename.c_str());
  ROS_INFO("Map has %dK points", map_->points.size()/1024);

  pcl::io::savePCDFileBinary<PointT>(request.filename, *map_);
  //pcl::io::savePLYFile<PointT>(request.filename + ".ply", *map_);

  ROS_INFO("Save successful");
  return true;
}

} // namespace ccny_rgbd
