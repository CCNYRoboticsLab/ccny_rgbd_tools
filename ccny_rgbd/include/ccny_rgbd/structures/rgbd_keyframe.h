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

    double max_data_range_;    // maximum z range for dense data
    double max_sigma_z_;
    double max_var_z_;
};

typedef Eigen::aligned_allocator<RGBDKeyframe> KeyframeAllocator;
typedef std::vector<RGBDKeyframe, KeyframeAllocator> KeyframeVector;

} //namespace ccny_rgbd


#endif // CCNY_RGBD_RGBD_KEYFRAME_H
