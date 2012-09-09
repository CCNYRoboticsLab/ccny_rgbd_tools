#ifndef CCNY_RGBD_LOOP_DETECTION_H
#define CCNY_RGBD_LOOP_DETECTION_H

#include <tf/transform_datatypes.h>
#include <boost/thread/mutex.hpp>

#include "ccny_rgbd/structures/rgbd_keyframe.h"
#include "ccny_rgbd/loop/keyframe_generator.h"

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

class LoopDetection
{
  public:

    LoopDetection(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~LoopDetection();

    void generateKeyframeAssociations(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    bool getRANSACMatching(
      RGBDFrame& frame_src, RGBDFrame& frame_dst, 
      float matching_distance, float eps_reproj, float inlier_threshold);

    void getRANSACInliers(
      RGBDFrame& frame_src, RGBDFrame& frame_dst, 
      float matching_distance, float eps_reproj,
      std::vector<cv::DMatch>& all_matches,
      std::vector<cv::DMatch>& inlier_matches);

  protected:
  
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

   private:

    boost::mutex mutex_;

    void prepareFeaturesForRANSAC(KeyframeVector& keyframes);

    void bruteForceAssociations();

    void ringAssociations(  
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    void treeAssociations(  
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_LOOP_DETECTION_H
