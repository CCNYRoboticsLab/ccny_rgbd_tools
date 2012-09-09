#ifndef CCNY_RGBD_RGBD_KEYFRAME_H
#define CCNY_RGBD_RGBD_KEYFRAME_H

#include <opencv2/nonfree/features2d.hpp>
#include <pcl_ros/transforms.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include "ccny_rgbd/structures/rgbd_frame.h"

namespace ccny_rgbd
{

class RGBDKeyframe: public RGBDFrame
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RGBDKeyframe(const RGBDFrame& frame);
  
    tf::Transform pose;
    PointCloudT   data;

    double path_length_linear;
    double path_length_angular;

    void constructDataCloud();

  private:

    float max_data_range_;    // maximum z range for dense data
};

} //namespace ccny_rgbd


#endif // CCNY_RGBD_RGBD_KEYFRAME_H
