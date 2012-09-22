#ifndef CCNY_RGBD_KEYFRAME_MAPPER_H
#define CCNY_RGBD_KEYFRAME_MAPPER_H

#include <ros/publisher.h>
#include <visualization_msgs/Marker.h>

#include "ccny_rgbd/mapping/keyframe_generator.h"
#include "ccny_rgbd/PublishKeyframe.h"
#include "ccny_rgbd/PublishAllKeyframes.h"
#include "ccny_rgbd/Recolor.h"

namespace ccny_rgbd
{

class KeyframeMapper: public KeyframeGenerator
{
  public:

    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyframeMapper(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeMapper();

    void RGBDCallback(const ImageMsg::ConstPtr& depth_msg,
                      const ImageMsg::ConstPtr& rgb_msg,
                      const CameraInfoMsg::ConstPtr& info_msg);

    bool publishAllKeyframesSrvCallback(PublishAllKeyframes::Request& request,
                                        PublishAllKeyframes::Response& response);

    bool publishKeyframeSrvCallback(PublishKeyframe::Request& request,
                                    PublishKeyframe::Response& response);

    bool recolorSrvCallback(Recolor::Request& request,
                            Recolor::Response& response);

  private:

    ros::Publisher keyframes_pub_;
    ros::Publisher poses_pub_;
    ros::Publisher edges_pub_;
    ros::ServiceServer pub_frames_service_;
    ros::ServiceServer pub_frame_service_;
    ros::ServiceServer recolor_service_;

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
