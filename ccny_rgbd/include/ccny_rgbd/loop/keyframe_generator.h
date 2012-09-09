#ifndef CCNY_RGBD_KEYFRAME_GENERATOR_H
#define CCNY_RGBD_KEYFRAME_GENERATOR_H

#include <stdio.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>
#include <visualization_msgs/Marker.h>

#include "ccny_rgbd/structures/rgbd_frame.h"
#include "ccny_rgbd/structures/rgbd_keyframe.h"
#include "ccny_rgbd/PublishFrame.h"
#include "ccny_rgbd/PublishAllFrames.h"

namespace ccny_rgbd
{

typedef Eigen::aligned_allocator<RGBDKeyframe> KeyframeAllocator;
typedef std::vector<RGBDKeyframe, KeyframeAllocator> KeyframeVector;

class KeyframeGenerator
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyframeGenerator(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeGenerator();
 
    void processFrame(const RGBDFrame& frame, const tf::Transform& pose);
    void addKeyframe(const RGBDFrame& frame, const tf::Transform& pose);

    bool publishAllFramesSrvCallback(ccny_rgbd::PublishAllFrames::Request& request,
                                     ccny_rgbd::PublishAllFrames::Response& response);

    bool publishFrameSrvCallback(ccny_rgbd::PublishFrame::Request& request,
                                 ccny_rgbd::PublishFrame::Response& response);

    void publishKeyframeData(int i);
    void publishKeyframePose(int i);
    void publishEdges();

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    KeyframeVector keyframes_;

    void publishMatchEdge(int i, int j);

    std::string fixed_frame_;

  private:

    ros::Publisher keyframes_pub_;
    ros::Publisher poses_pub_;
    ros::Publisher edges_pub_;
    ros::ServiceServer pub_frames_service_;
    ros::ServiceServer pub_frame_service_;

    // params
    
    double kf_dist_eps_;
    double kf_angle_eps_;
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_GENERATOR_H
