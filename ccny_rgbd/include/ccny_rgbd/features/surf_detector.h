#ifndef CCNY_RGBD_SURF_DETECTOR_H
#define CCNY_RGBD_SURF_DETECTOR_H

#include <opencv2/nonfree/features2d.hpp>

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

class SurfDetector: public FeatureDetector
{
  public:

    SurfDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~SurfDetector();

    void findFeatures(RGBDFrame& frame, const cv::Mat * input_img);

  private:

    double hessian_threshold_;
    bool upright_;

    cv::SurfDescriptorExtractor surf_descriptor_;
    cv::SurfFeatureDetector * surf_detector_;
};

} //namespace ccny_rgbd



#endif // CCNY_RGBD_SURF_DETECTOR_H
