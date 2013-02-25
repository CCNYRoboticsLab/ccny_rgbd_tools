#ifndef CCNY_RGBD_CALIBRATE_CALIB_UTIL_H
#define CCNY_RGBD_CALIBRATE_CALIB_UTIL_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

#include <ccny_rgbd/rgbd_util.h>

namespace ccny_rgbd {
    
cv::Mat m4(const cv::Mat& m3);

cv::Mat matrixFromRT(const cv::Mat& rmat, const cv::Mat& tvec);

cv::Mat matrixFromRvecTvec(const cv::Mat& rvec, const cv::Mat& tvec);
  
/* overlays an RGB and depth image, used for 
 * testing registration
 */
void blendImages(const cv::Mat& rgb_img,
                 const cv::Mat depth_img,
                 cv::Mat& blend_img);
  
/* creates an 8-bit depth image from a 
 * 16-bit depth image in mm
 */
void create8bImage(
  const cv::Mat depth_img,
  cv::Mat& depth_img_u);
  
/* Given an image and a pattern size, detects and
 * returns a vector of checkerboard corners 
 */
bool getCorners(
  const cv::Mat& img,
  const cv::Size& pattern_size,
  std::vector<cv::Point2f>& corners);

/* Given a depth and an rgb image, it blends them
 * and displays the result 
 */
void showBlendedImage(
  const cv::Mat& depth_img,
  const cv::Mat& rgb_img,
  const std::string& title);

/* displays an image with checkerboard corners
 */
void showCornersImage(
  const cv::Mat& img, 
  const cv::Size& pattern_size, 
  const Point2fVector& corners_2d, 
  bool corner_result,
  const std::string title);

/* Given a set of detected checkerboard corners,
 * outputs the 4 vertices of a rectangle which describes
 * the checkerboard, plus a small margin around it
 * to caprure the border squares
 */
void getCheckerBoardPolygon(
  const Point2fVector& corners_2d,
  int n_rows, int n_cols,
  Point2fVector& vertices);

/* given a set of vertices describing the outline of a checkerboard,
 * the intrinsics of the rgb camera, and the extrinsic matrix between
 * the camera and the checkerboard, builds a depth image of the 
 * checkerboard 
 */
void buildCheckerboardDepthImage(
  const Point3fVector& corners_3d_depth,
  const Point2fVector& vertices,
  const cv::Mat& intr_rect_depth,
  cv::Mat& depth_img);

} // namespace ccny_rgbd

#endif // CCNY_RGBD_CALIBRATE_CALIB_UTIL_H
