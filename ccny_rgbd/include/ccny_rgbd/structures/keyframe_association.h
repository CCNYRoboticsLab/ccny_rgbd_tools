#ifndef CCNY_RGBD_KEYFRAME_ASSOCIATION_H
#define CCNY_RGBD_KEYFRAME_ASSOCIATION_H

#include <Eigen/StdVector>
#include <opencv2/features2d/features2d.hpp>

namespace ccny_rgbd
{

struct KeyframeAssociation
{
  int kf_idx_a;                    // index of keyframe A
  int kf_idx_b;                    // index of keyframe B
  std::vector<cv::DMatch> matches; // vector of RANSAC inliers
  tf::Transform a2b;               // ???
};

typedef Eigen::aligned_allocator<KeyframeAssociation> KeyframeAssociationAllocator;
typedef std::vector<KeyframeAssociation, KeyframeAssociationAllocator> KeyframeAssociationVector;

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_ASSOCIATION_H
