#ifndef CCNY_RGBD_CALIBRATE_CALIB_UTIL_H
#define CCNY_RGBD_CALIBRATE_CALIB_UTIL_H

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace ccny_rgbd {
    
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef std::vector<cv::Point2f> Point2fVector;
typedef std::vector<cv::Point3f> Point3fVector;
  
void buildRegisteredDepthImage(
  const cv::Mat& intr_rect_ir,
  const cv::Mat& intr_rect_rgb,
  const cv::Mat& rgb2ir,
  const cv::Mat& depth_img_rect,
  cv::Mat& depth_img_rect_reg);

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

void create8bImage(
  const cv::Mat depth_img,
  cv::Mat& depth_img_u);
  

bool getCorners(
  const cv::Mat& img,
  const cv::Size& pattern_size,
  std::vector<cv::Point2f>& corners);

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

} // namespace ccny_rgbd


#endif // CCNY_RGBD_CALIBRATE_CALIB_UTIL_H