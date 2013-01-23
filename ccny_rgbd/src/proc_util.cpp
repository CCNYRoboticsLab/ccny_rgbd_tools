#include "ccny_rgbd/proc_util.h"

namespace ccny_rgbd {

void unwarpDepthImage(
  cv::Mat& depth_img,
  const cv::Mat& coeff0,
  const cv::Mat& coeff1,
  const cv::Mat& coeff2,
  int fit_mode)
{
  /*
  // TODO: This is faster (5ms vs 4 ms)
  // BUT images need to be passed as CV_64FC1 or CV_32FC1
  // currently the conversion to float messes up 0 <-> nan values
  // Alternatively, coeff0 needs to be 0
  
  cv::Mat temp;
  depth_img.convertTo(temp, CV_64FC1);
  temp = coeff0 + temp.mul(coeff1 + temp.mul(coeff2));   
  temp.convertTo(depth_img, CV_16UC1);
  */

  if (fit_mode == DEPTH_FIT_QUADRATIC)
  {
    for (int u = 0; u < depth_img.cols; ++u)
    for (int v = 0; v < depth_img.rows; ++v)    
    {
      uint16_t d = depth_img.at<uint16_t>(v, u);    
      if (d != 0)
      {
        double c0 = coeff0.at<double>(v, u);
        double c1 = coeff1.at<double>(v, u);
        double c2 = coeff2.at<double>(v, u);
      
        uint16_t res = (int)(c0 + d*(c1 + d*c2));
        depth_img.at<uint16_t>(v, u) = res;
      }
    }
  }
  else if(fit_mode == DEPTH_FIT_LINEAR)
  {
    for (int u = 0; u < depth_img.cols; ++u)
    for (int v = 0; v < depth_img.rows; ++v)    
    {
      uint16_t d = depth_img.at<uint16_t>(v, u);    
      if (d != 0)
      {
        double c0 = coeff0.at<double>(v, u);
        double c1 = coeff1.at<double>(v, u);
      
        uint16_t res = (int)(c0 + d*c1);
        depth_img.at<uint16_t>(v, u) = res;
      }
    }
  }
  else if (fit_mode == DEPTH_FIT_LINEAR_ZERO)
  {
    cv::Mat temp;
    depth_img.convertTo(temp, CV_64FC1);
    temp = temp.mul(coeff1);   
    temp.convertTo(depth_img, CV_16UC1);
  }
  else if (fit_mode == DEPTH_FIT_QUADRATIC_ZERO)
  {
    cv::Mat temp;
    depth_img.convertTo(temp, CV_64FC1);
    temp = temp.mul(coeff1 + temp.mul(coeff2));   
    temp.convertTo(depth_img, CV_16UC1);
  }
}

} // namespace ccny_rgbd