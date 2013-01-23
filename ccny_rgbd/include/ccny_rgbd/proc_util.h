#ifndef CCNY_RGBD_PROC_UTIL_H
#define CCNY_RGBD_PROC_UTIL_H

#include <opencv2/opencv.hpp>

namespace ccny_rgbd {

enum DepthFitMode { 
  DEPTH_FIT_LINEAR,
  DEPTH_FIT_LINEAR_ZERO,
  DEPTH_FIT_QUADRATIC,
  DEPTH_FIT_QUADRATIC_ZERO
};

/* given a depth image, uwarps it according to a polynomial model
 * d = c0 + c1*d + c2*d^2
 */
void unwarpDepthImage(
  cv::Mat& depth_img_in,
  const cv::Mat& coeff0,
  const cv::Mat& coeff1,
  const cv::Mat& coeff2,
  int fit_mode=DEPTH_FIT_QUADRATIC);

} // namespace ccny_rgbd

#endif // CCNY_RGBD_PROC_UTIL_H
