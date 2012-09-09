#ifndef CCNY_3D_MAP_SERVER_MAP_SERVER_H
#define CCNY_3D_MAP_SERVER_MAP_SERVER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <ccny_gicp/octree_pointcloud_storage.h>
#include <ccny_gicp/octree_pointcloud_storage.hpp>

#include "ccny_3dmap_server/Save.h"

namespace ccny_rgbd
{

typedef pcl::PointXYZRGB        PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const std::string sub_topic_  = "/camera/depth_registered/points";
const std::string map_topic_  = "/map";

class MapServer
{
  public:

    typedef pcl::octree::OctreePointCloudStorage<PointT> Octree;

    MapServer(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~MapServer();

    void pointCloudCallback(const PointCloudT::ConstPtr& cloud_in_ptr);

    bool saveSrvCallback(ccny_3dmap_server::Save::Request& request,
                         ccny_3dmap_server::Save::Response& response);


  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber point_cloud_subscriber_;
    ros::Publisher map_pub_;

    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::ServiceServer save_service_;

    // **** parameters

    std::string fixed_frame_;
    double resolution_;
    double max_range_;

    // **** variables

    unsigned int cloud_count_;

    boost::mutex mutex_;

    Octree * octree_;
    PointCloudT::Ptr map_;

    // **** private functions

    void initParams();

    void publishMap();

    void filterCloud(const PointCloudT::ConstPtr& cloud_in_ptr,
                     const PointCloudT::Ptr& data_ptr);
};

} //namespace ccny_rgbd

#endif // CCNY_3D_MAP_SERVER_MAP_SERVER_H
