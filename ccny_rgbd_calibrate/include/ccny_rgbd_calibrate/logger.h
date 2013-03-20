#ifndef CCNY_RGBD_CALIBRATE_LOGGER_H
#define CCNY_RGBD_CALIBRATE_LOGGER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/filesystem.hpp>

#include <ccny_rgbd/structures/rgbd_frame.h>
#include <ccny_rgbd/types.h>

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
    
    void IRCallback(const sensor_msgs::ImageConstPtr& ir_msg);

  private:

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    boost::shared_ptr<RGBDSynchronizer3> sync_;
       
    ImageSubFilter      sub_depth_;
    ImageSubFilter      sub_rgb_;
    ros::Subscriber     sub_ir_;
    CameraInfoSubFilter sub_info_;

    std::string rgb_path_;
    std::string depth_path_;
    std::string ir_path_;

    int id_;

    // parameters   
    int n_rgb_;
    int n_depth_;
    int n_ir_;
   
    std::string path_;
    
    void prepareDirectories();
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_CALIBRATE_LOGGER_H
