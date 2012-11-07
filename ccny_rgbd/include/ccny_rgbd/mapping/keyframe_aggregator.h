#ifndef CCNY_RGBD_KEYFRAME_AGGREGATOR_H
#define CCNY_RGBD_KEYFRAME_AGGREGATOR_H

#include <stdio.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/point_cloud.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/structures/rgbd_frame.h"
#include "ccny_rgbd/structures/rgbd_keyframe.h"

namespace ccny_rgbd
{

class KeyframeAggregator
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    KeyframeAggregator(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeAggregator();

    void processKeyframe(const RGBDKeyframe& keyframe);  

  private:

    PointCloudT model_;

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    ros::Publisher map_pub_;

    int id_; // keyframe counter

    void projectModelToImage(const RGBDKeyframe& keyframe);

    void createScaleImage(const cv::Mat& mask_img, 
                          cv::Mat& scale_img);

    void projectCloudToImage(const PointCloudT& scene,  
                             const cv::Mat& M,
                             cv::Mat& projected_img,
                             cv::Mat& idx_img,
                             cv::Mat& mask_img);

    void blendImages(const cv::Mat& data_img, 
                     const cv::Mat& proj_img,
                     const cv::Mat& scale_img,
                     cv::Mat& blend_img);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_AGGREGATOR_H
