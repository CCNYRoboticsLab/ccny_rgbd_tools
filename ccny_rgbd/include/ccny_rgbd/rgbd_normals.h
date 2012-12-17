#ifndef CCNY_RGBD_RGBD_NORMALS_H
#define CCNY_RGBD_RGBD_NORMALS_H

#include "ccny_rgbd/rgbd_util.h"
#include "ccny_rgbd/structures/rgbd_frame.h"

namespace ccny_rgbd {

void g();

void f(const RGBDFrame& frame);

void buildPointCloud(
  const cv::Mat& depth_img_rect_reg,
  const cv::Mat& rgb_img_rect,
  const cv::Mat& intr_rect_rgb,
  PointCloudT& cloud);

void create2DProjectionImage(
  const PointCloudT& cloud,
  cv::Mat& img,
  double min_z = -std::numeric_limits<double>::infinity(),
  double max_z = std::numeric_limits<double>::infinity());

void filterCloudByHeight(
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_in,
  pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_out,
  double min_z,
  double max_z);

void normalizeHistogram(cv::Mat& histogram);

void create8bitHistogram(
  const cv::Mat& histogram,
  cv::Mat& histogram_norm);

void createImageFromHistogram(
  const cv::Mat& histogram,
  cv::Mat& image);

void shiftHistogram(
  const cv::Mat& hist_in,
  cv::Mat& hist_out,
  int bins);

void buildExpectedPhiHistorgtam(
  cv::Mat& histogram,
  double degrees_per_bin,
  double stdev);

void buildPhiHistogram(
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud,
  cv::Mat& histogram,
  double degrees_per_bin);

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_NORMALS_H
