#ifndef CCNY_RGBD_SIFT_GPU_DETECTOR_H
#define CCNY_RGBD_SIFT_GPU_DETECTOR_H

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/gpu/gpu.hpp>

#include "ccny_rgbd/features/feature_detector.h"
#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

class SiftGpuDetector: public FeatureDetector
{
  public:

    SiftGpuDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~SiftGpuDetector();

    void findFeatures(RGBDFrame& frame, const cv::Mat * input_img);

  private:

    //int n_features_;
    //double min_distance_;
    //int grid_cells_;

    //cv::GoodFeaturesToTrackDetector * gft_detector_;
    cv::gpu::SURF_GPU surf_gpu_detector_;
};

} //namespace ccny_rgbd



#endif // CCNY_RGBD_SIFT_GPU_DETECTOR_H
