/*
 *  Copyright (C) 2013, City University of New York
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *
 *  CCNY Robotics Lab
 *  http://robotics.ccny.cuny.edu
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

#ifndef CCNY_RGBD_PROC_UTIL_H
#define CCNY_RGBD_PROC_UTIL_H

#include <opencv2/opencv.hpp>

namespace ccny_rgbd {

/** @brief Polynomial fit modes for depth unwarping
 * 
 * The modes include:
 *  - DEPTH_FIT_LINEAR (c0 + c1*d)
 *  - DEPTH_FIT_LINEAR_ZERO (c1*d)
 *  - DEPTH_FIT_QUADRATIC (c0 + c1*d + c2*d^2)
 *  - DEPTH_FIT_QUADRATIC_ZERO (c1*d + c2*d^2)
 *
 * The recommended mode is DEPTH_FIT_QUADRATIC
 */
enum DepthFitMode { 
  DEPTH_FIT_LINEAR,
  DEPTH_FIT_LINEAR_ZERO,
  DEPTH_FIT_QUADRATIC,
  DEPTH_FIT_QUADRATIC_ZERO
};

/** @brief Given a depth image, uwarps it according to a polynomial model
 * 
 * The size of the c matrices should be equal to the image size.
 * 
 * @param depth_img_in depth image to be unwarped (16UC1, in mm)
 * @param coeff0 matrix of c0 coefficients
 * @param coeff1 matrix of c1 coefficients
 * @param coeff2 matrix of c2 coefficients
 * @param fit_mode the polynomial fitting mode, see \ref DepthFitMode.
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
