#ifndef CCNY_RGBD_FAST_DETECTOR_H
#define CCNY_RGBD_FAST_DETECTOR_H

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

class FastDetector: public FeatureDetector
{
  public:

    FastDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~FastDetector();

    void findFeatures(RGBDFrame& frame, const cv::Mat * input_img);

  private:

    int threshold_;
    bool nonmax_suppression_;
    
    cv::FastFeatureDetector * fast_detector_;
    cv::OrbDescriptorExtractor orb_descriptor_;
};

} //namespace ccny_rgbd



#endif // CCNY_RGBD_FAST_DETECTOR_H
