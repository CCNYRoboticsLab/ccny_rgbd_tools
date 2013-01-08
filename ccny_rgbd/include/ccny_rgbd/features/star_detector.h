#ifndef CCNY_RGBD_STAR_DETECTOR_H
#define CCNY_RGBD_STAR_DETECTOR_H

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

class StarDetector: public FeatureDetector
{
  public:

    StarDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~StarDetector();

    /*
    void setThreshold(int threshold);
    int getThreshold() const;
    void setNFeatures(unsigned int n_features);
    unsigned int getNFeatures() const;
    */

    void findFeatures(RGBDFrame& frame, const cv::Mat& input_img);

  private:

    //int n_features_;
    //double edge_threshold_; 

    cv::FeatureDetector * star_detector_;
};

} //namespace ccny_rgbd



#endif // CCNY_RGBD_STAR_DETECTOR_H
