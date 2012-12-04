#ifndef CCNY_RGBD_CALIBRATE_CALIB_UTIL_H
#define CCNY_RGBD_CALIBRATE_CALIB_UTIL_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

namespace ccny_rgbd {
    
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef std::vector<cv::Point2f> Point2fVector;
typedef std::vector<cv::Point3f> Point3fVector;
  
/* reprojects a depth image to another depth image,
 * registered in the rgb camera's frame
 */

void buildRegisteredDepthImage(
  const cv::Mat& intr_rect_ir,
  const cv::Mat& intr_rect_rgb,
  const cv::Mat& rgb2ir,
  const cv::Mat& depth_img_rect,
  cv::Mat& depth_img_rect_reg);

/*
 * Constructs a point cloud, given a depth image which
 * has been undistorted and registered in the rgb frame, 
 * and an rgb image.
 * The intinsic matrix is the RGB matrix after rectification
 * The depth image is uint16_t, in mm
 */
void buildPointCloud(
  const cv::Mat& depth_img_rect_reg,
  const cv::Mat& rgb_img_rect,
  const cv::Mat& intr_rect_rgb,
  PointCloudT& cloud);
  
  
void blendImages(const cv::Mat& rgb_img,
                 const cv::Mat depth_img,
                 cv::Mat& blend_img);
  
void matrixFromRvecTvec(const cv::Mat& rvec,
                        const cv::Mat& tvec,
                        cv::Mat& E);

void matrixFromRT(const cv::Mat& rmat,
                  const cv::Mat& tvec,
                  cv::Mat& E);

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

void showCornersImage(
  const cv::Mat& img, 
  const cv::Size pattern_size, 
  const Point2fVector corners_2d, 
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
  const Point3fVector& corners_3d,
  const Point2fVector& vertices,
  int width, int height,
  const cv::Mat& rvec, const cv::Mat& tvec,
  const cv::Mat& intr_rect_rgb,
  cv::Mat& depth_img);

/* given a depth image, uwarps it according to a polynomial model
 * d = c0 + c1*d + c2*d^2
 */

void unwarpDepthImage(
  cv::Mat& depth_img_in,
  const cv::Mat& coeff0,
  const cv::Mat& coeff1,
  const cv::Mat& coeff2);

} // namespace ccny_rgbd


#endif // CCNY_RGBD_CALIBRATE_CALIB_UTIL_H