#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include "ccny_rgbd/types.h"
#include "ccny_rgbd/rgbd_map_util.h"

using namespace ccny_rgbd;

int main (int argc, char** argv)
{
  // read in
  printf("Reading cloud\n");
  PointCloudT::Ptr cloud;
  cloud.reset(new PointCloudT());
  pcl::PCDReader reader;
  reader.read (argv[1], *cloud);

    // show map
  printf("Showing map\n");
  cv::Mat map;
  create2DProjectionImage(*cloud, map);
  cv::imshow("map", map);
  cv::waitKey(0);
  
  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
 
  //mls.setComputeNormals (true);

  printf("MLS\n");
  
  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.05);

  // Reconstruct
  mls.reconstruct(*cloud);

    // show map
  printf("Showing map\n");
  cv::Mat map_mls;
  create2DProjectionImage(*cloud, map_mls);
  cv::imshow("map_mls", map_mls);
  cv::waitKey(0);
  
  printf("Saving\n");
  
  // Save output
  pcl::io::savePCDFile ("/home/idyanov/ros/office_loop_mls.pcd", *cloud);
}