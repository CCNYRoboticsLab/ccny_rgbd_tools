#ifndef CCNY_RGBD_AVG_LOGGER_H
#define CCNY_RGBD_AVG_LOGGER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/filesystem.hpp>

#include "ccny_rgbd/structures/rgbd_frame.h"
#include "ccny_rgbd/types.h"

namespace ccny_rgbd
{

class AvgLogger 
{
  public:

    AvgLogger(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~AvgLogger();

    void RGBDCallback(const ImageMsg::ConstPtr& depth_msg,
                      const ImageMsg::ConstPtr& rgb_msg,
                      const CameraInfoMsg::ConstPtr& info_msg);  

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    boost::shared_ptr<Synchronizer> sync_;
       
    ImageSubFilter      sub_depth_;
    ImageSubFilter      sub_rgb_;
    CameraInfoSubFilter sub_info_;

    std::stringstream ss_rgb_path_;
    std::stringstream ss_depth_path_;
    std::stringstream ss_filename_;

    int count_;

    cv::Mat depth_cnt_img_;
    cv::Mat depth_sum_img_;
    
    // parameters   
    int n_depth_;
    int id_;   
    std::string sequence_;

    void prepareDirectories();
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_AVG_LOGGER_H
