#ifndef CCNY_RGBD_LOGGER_H
#define CCNY_RGBD_LOGGER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/filesystem.hpp>

#include "ccny_rgbd/structures/rgbd_frame.h"
#include "ccny_rgbd/types.h"

namespace ccny_rgbd
{

class Logger 
{
  public:

    Logger(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~Logger();

    void RGBDCallback(const ImageMsg::ConstPtr& depth_msg,
                      const ImageMsg::ConstPtr& rgb_msg,
                      const CameraInfoMsg::ConstPtr& info_msg);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    boost::shared_ptr<image_transport::ImageTransport> rgb_it_;
    boost::shared_ptr<image_transport::ImageTransport> depth_it_;
    boost::shared_ptr<Synchronizer> sync_;
       
    ImageSubFilter      sub_depth_;
    ImageSubFilter      sub_rgb_;
    CameraInfoSubFilter sub_info_;

    std::stringstream ss_rgb_path_;
    std::stringstream ss_depth_path_;

    int id_;

    int n_;
    std::string sequence_;

    void prepareDirectories();
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_LOGGER_H
