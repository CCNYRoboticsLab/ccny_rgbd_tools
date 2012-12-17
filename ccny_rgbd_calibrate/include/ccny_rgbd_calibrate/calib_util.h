#ifndef CCNY_RGBD_CALIBRATE_CALIB_UTIL_H
#define CCNY_RGBD_CALIBRATE_CALIB_UTIL_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

#include <ccny_rgbd/rgbd_util.h>

namespace ccny_rgbd {
    
enum DepthFitMode { 
  DEPTH_FIT_LINEAR,
  DEPTH_FIT_LINEAR_ZERO,
  DEPTH_FIT_QUADRATIC,
  DEPTH_FIT_QUADRATIC_ZERO
};

cv::Mat m4(const cv::Mat& m3);

cv::Mat matrixFromRT(const cv::Mat& rmat, const cv::Mat& tvec);

cv::Mat matrixFromRvecTvec(const cv::Mat& rvec, const cv::Mat& tvec);

/* reprojects a depth image to another depth image,
 * registered in the rgb camera's frame. Both images 
 * need to be rectified first. ir2rgb is a matrix such that 
 * for any point P_IR in the depth camera frame
 * P_RGB = ir2rgb * P
 */
void buildRegisteredDepthImage(
  const cv::Mat& intr_rect_ir,
  const cv::Mat& intr_rect_rgb,
  const cv::Mat& ir2rgb,
  const cv::Mat& depth_img_rect,
  cv::Mat& depth_img_rect_reg);

/* Constructs a point cloud, given:
 *  - a depth image (uint16_t, mm) which has been undistorted
 *  - the intinsic matrix of the depth image after rectification
 */
void buildPointCloud(
  const cv::Mat& depth_img_rect,
  const cv::Mat& intr_rect_ir,
  PointCloudT& cloud);

/* Constructs a point cloud with color, given: 
 *  - a depth image (uint16_t, mm) which has been undistorted 
 *    and registeredin the rgb frame, 
 *  - an an rgb image which has been undistorted
 *  - the intinsic matrix of the RGB image after rectification
 */
void buildPointCloud(
  const cv::Mat& depth_img_rect_reg,
  const cv::Mat& rgb_img_rect,
  const cv::Mat& intr_rect_rgb,
  PointCloudT& cloud);
  
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


#endif // CCNY_RGBD_CALIBRATE_CALIB_UTIL_H
