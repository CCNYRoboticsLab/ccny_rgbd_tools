#ifndef CCNY_RGBD_KEYFRAME_ASSOCIATION_H
#define CCNY_RGBD_KEYFRAME_ASSOCIATION_H

#include <Eigen/StdVector>
#include <opencv2/features2d/features2d.hpp>

namespace ccny_rgbd
{

enum KeyframeAssociationType {VO, RANSAC, ODOMETRY};  
  
class KeyframeAssociation
{
  public:
 
    enum Type {VO, RANSAC, ODOMETRY};  
    
    int kf_idx_a;                    // index of keyframe A
    int kf_idx_b;                    // index of keyframe B
    
    Type type;    // source of the association
    
    std::vector<cv::DMatch> matches; // for type=RANSAC, vector of RANSAC inliers
    
    tf::Transform a2b;               // ???
};

typedef Eigen::aligned_allocator<KeyframeAssociation> KeyframeAssociationAllocator;
typedef std::vector<KeyframeAssociation, KeyframeAssociationAllocator> KeyframeAssociationVector;

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_ASSOCIATION_H
