#ifndef CCNY_RGBD_RGBD_MAP_UTIL_H
#define CCNY_RGBD_RGBD_MAP_UTIL_H

#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/rgbd_util.h"
#include "ccny_rgbd/structures/rgbd_keyframe.h"
#include "ccny_rgbd/structures/rgbd_frame.h"

namespace ccny_rgbd {

void alignGlobalMap(const PointCloudT::Ptr& cloud);

void buildExpectedPhiHistorgtam(
  cv::Mat& histogram,
  double degrees_per_bin,
  double stdev);

void buildPhiHistogram(
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud,
  cv::Mat& histogram,
  double degrees_per_bin);

void normalizeHistogram(cv::Mat& histogram);

void shiftHistogram(
  const cv::Mat& hist_in,
  cv::Mat& hist_out,
  int bins);

bool alignHistogram(
  const cv::Mat& hist,
  const cv::Mat& hist_exp,
  double hist_resolution,
  double& best_angle);

void create8bitHistogram(
  const cv::Mat& histogram,
  cv::Mat& histogram_norm);

void createImageFromHistogram(
  const cv::Mat& histogram,
  cv::Mat& image);

void create2DProjectionImage(
  const PointCloudT& cloud, 
  cv::Mat& img,
  double min_z = -std::numeric_limits<double>::infinity(),
  double max_z =  std::numeric_limits<double>::infinity());  

void filterCloudByHeight(
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_in,
  pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_out,
  double min_z,
  double max_z);

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_MAP_UTIL_H
