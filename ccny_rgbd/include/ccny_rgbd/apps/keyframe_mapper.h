#ifndef CCNY_RGBD_KEYFRAME_MAPPER_H
#define CCNY_RGBD_KEYFRAME_MAPPER_H

#include <ros/ros.h>
#include <ros/publisher.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "ccny_rgbd/structures/rgbd_keyframe.h"
#include "ccny_rgbd/mapping/keyframe_generator.h"
#include "ccny_rgbd/PublishKeyframe.h"
#include "ccny_rgbd/PublishAllKeyframes.h"
#include "ccny_rgbd/Recolor.h"
#include "ccny_rgbd/Save.h"

namespace ccny_rgbd
{

class KeyframeMapper: public KeyframeGenerator
{
  public:

    KeyframeMapper(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeMapper();

    bool publishAllKeyframesSrvCallback(PublishAllKeyframes::Request& request,
                                        PublishAllKeyframes::Response& response);

    bool publishKeyframeSrvCallback(PublishKeyframe::Request& request,
                                    PublishKeyframe::Response& response);

    bool recolorSrvCallback(Recolor::Request& request,
                            Recolor::Response& response);

    bool saveKeyframesSrvCallback(Save::Request& request,
                                  Save::Response& response);

    bool saveKeyframesFFSrvCallback(Save::Request& request,
                                    Save::Response& response);

  protected:

    virtual void RGBDCallback(
      const ImageMsg::ConstPtr& depth_msg,
      const ImageMsg::ConstPtr& rgb_msg,
      const CameraInfoMsg::ConstPtr& info_msg);

  private:

    ros::Publisher keyframes_pub_;
    ros::Publisher poses_pub_;
    ros::Publisher edges_pub_;
    ros::ServiceServer pub_frames_service_;
    ros::ServiceServer pub_frame_service_;
    ros::ServiceServer recolor_service_;
    ros::ServiceServer save_kf_service_;
    ros::ServiceServer save_kf_ff_service_;

    tf::TransformListener tf_listener_;

    boost::shared_ptr<image_transport::ImageTransport> rgb_it_;
    boost::shared_ptr<image_transport::ImageTransport> depth_it_;
    boost::shared_ptr<Synchronizer> sync_;
       
    ImageSubFilter      sub_depth_;
    ImageSubFilter      sub_rgb_;
    CameraInfoSubFilter sub_info_;

    void publishMatchEdge(int i, int j);
    void publishKeyframeData(int i);
    void publishKeyframePose(int i);
    void publishEdges();
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_MAPPER_H
