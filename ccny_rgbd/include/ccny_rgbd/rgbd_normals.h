#ifndef CCNY_RGBD_RGBD_NORMALS_H
#define CCNY_RGBD_RGBD_NORMALS_H

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include "ccny_rgbd/rgbd_util.h"
#include "ccny_rgbd/rgbd_map_util.h"

namespace ccny_rgbd {

void analyzeKeyframes();

bool analyzeCloud(
  const PointCloudT::Ptr& cloud,
  double& best_angle);

void analyzeKeyframe(RGBDKeyframe& keyframe);

} // namespace ccny_rgbd

#endif // CCNY_RGBD_RGBD_NORMALS_H
