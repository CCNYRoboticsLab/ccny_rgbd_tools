#ifndef CCNY_RGBD_OCTREE_MAPPER_H
#define CCNY_RGBD_OCTREE_MAPPER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/structures/rgbd_frame.h"
#include "ccny_rgbd/structures/rgbd_keyframe.h"
#include "ccny_rgbd/mapping/octree_pointcloud_storage.h"
#include "ccny_rgbd/mapping/octree_pointcloud_storage.hpp"

#include "ccny_rgbd/Save.h"

namespace ccny_rgbd
{

typedef pcl::PointXYZRGB        PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef pcl::octree::OctreeLeafDataT<int> LeafT;
typedef pcl::octree::OctreeBase<int, LeafT> OctreeT;

typedef pcl::octree::OctreePointCloudStorage<PointT, LeafT, OctreeT> Octree;

typedef visualization_msgs::Marker      MarkerMsg;
typedef visualization_msgs::MarkerArray MarkerArrayMsg;

class OctreeMapper
{
  public:

    OctreeMapper(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~OctreeMapper();

    bool saveSrvCallback(ccny_rgbd::Save::Request& request,
                         ccny_rgbd::Save::Response& response);

    void RGBDCallback(const ImageMsg::ConstPtr& depth_msg,
                      const ImageMsg::ConstPtr& rgb_msg,
                      const CameraInfoMsg::ConstPtr& info_msg);

  private:

    // **** ROS-related

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Subscriber point_cloud_subscriber_;
    ros::Publisher map_pub_;
    ros::Publisher marker_pub_;

    tf::TransformListener    tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::ServiceServer save_service_;

    boost::shared_ptr<image_transport::ImageTransport> rgb_it_;
    boost::shared_ptr<image_transport::ImageTransport> depth_it_;
    boost::shared_ptr<Synchronizer> sync_;
       
    ImageSubFilter      sub_depth_;
    ImageSubFilter      sub_rgb_;
    CameraInfoSubFilter sub_info_;

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

    void publishPointMap();
    void publishMarkerMap();
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_OCTREE_MAPPER_H
