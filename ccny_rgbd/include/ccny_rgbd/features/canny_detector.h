#ifndef CCNY_RGBD_CANNY_DETECTOR_H
#define CCNY_RGBD_CANNY_DETECTOR_H

#include "ccny_rgbd/util.h"
#include "ccny_rgbd/features/feature_detector.h"

namespace ccny_rgbd
{

class CannyDetector : public FeatureDetector
{
  public:

    CannyDetector();
    ~CannyDetector();

    void findFeatures(RGBDFrame& frame, const cv::Mat * input_img);

    void setThreshold1(int threshold1);
    void setThreshold2(int threshold2);

    int getThreshold1() const;
    int getThreshold2() const;

    //void showGradientImage();

  private:

    int threshold1_;
    int threshold2_;

    cv::Mat img_angles_;
};

} //namespace ccny_rgbd


#endif // CCNY_RGBD_CANNY_DETECTOR_H
