/**
 *  @file proc_util.cpp
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2013, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

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
  // NOTE: This is slightly faster (5ms vs 4ms)
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