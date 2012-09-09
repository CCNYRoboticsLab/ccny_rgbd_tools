#ifndef CCNY_RGBD_KLT_DETECTOR_H
#define CCNY_RGBD_KLT_DETECTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

class KltDetector: public FeatureDetector
{

  public:

    KltDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~KltDetector();

    void findFeatures(RGBDFrame& frame, const cv::Mat * input_img);

  private:

    bool initialized_;
    bool have_points_;

    int n_features_;
    float reseed_threshold_;
    int win_size_;

    cv::Mat prev_input_img_;

    CvPoint2D32f * pointsA_;
    CvPoint2D32f * points_good_;

    IplImage * pyrA;
    IplImage * pyrB;

    cv::GoodFeaturesToTrackDetector * gft_detector_;
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_KLT_DETECTOR_H

