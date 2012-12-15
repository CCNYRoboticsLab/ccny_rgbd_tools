#ifndef CCNY_RGBD_RGBD_KEYFRAME_H
#define CCNY_RGBD_RGBD_KEYFRAME_H

#include "ccny_rgbd/structures/rgbd_frame.h"

namespace ccny_rgbd
{

class RGBDKeyframe: public RGBDFrame
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RGBDKeyframe(const RGBDFrame& frame);
  
    tf::Transform pose;
    PointCloudT   cloud;

    bool manually_added;

    double path_length_linear;
    double path_length_angular;

    void constructDensePointCloud(
      double max_z = 5.5,
      double max_stdev_z = 0.03);
};

typedef Eigen::aligned_allocator<RGBDKeyframe> KeyframeAllocator;
typedef std::vector<RGBDKeyframe, KeyframeAllocator> KeyframeVector;

} //namespace ccny_rgbd


#endif // CCNY_RGBD_RGBD_KEYFRAME_H
