#ifndef CCNY_RGBD_GFT_DETECTOR_H
#define CCNY_RGBD_GFT_DETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

class GftDetector: public FeatureDetector
{
  public:

    GftDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~GftDetector();

    void findFeatures(RGBDFrame& frame, const cv::Mat& input_img);

  private:

    int n_features_;
    double min_distance_;
    int grid_cells_;

    cv::GoodFeaturesToTrackDetector * gft_detector_;
    cv::GridAdaptedFeatureDetector * grid_detector_;
};

} //namespace ccny_rgbd



#endif // CCNY_RGBD_GFT_DETECTOR_H
