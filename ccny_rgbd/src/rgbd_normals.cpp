#include "ccny_rgbd/rgbd_normals.h"
#include "ccny_rgbd/rgbd_map_util.h"

/* 

-------------------------------------------------------------
global alignement idea:
-------------------------------------------------------------
 1) cloud is rp aligned, but arbitrary yaw
 2) histogram phi angle of all points
 3) recover x-y vectors which best fit histogram
   - possibly phase of histagram? period of 90 deg)
 4) rotate entire cloud to be yaw-aligned
 5) go through all keyframes and find planes
 6) for each plane determine if it's a wall
   - wall is any plane which is vertical (within tolerance)
     and also its yaw = 0 / 90 / 180 / 270 deg (within tolereance)
 7) for any wall, introduce a global yaw rotation "observation"
   - observation aligns it to 0 / 90 / 180 / 270
 8) minimize graph error
 9) possibly repeat 5 to 8 several times, as more and more 
    walls snap into place
*/

namespace ccny_rgbd {

void analyzeKeyframes()
{
/*
  KeyframeVector keyframes;
  loadKeyframes(keyframes, "/home/idyanov/ros/images/ccny_3rooms_seq_loop");

  for (unsigned int i = 0; i < keyframes.size(); i = i+10)
  {
    PointCloudT::Ptr agr(new PointCloudT);
    
    for (unsigned int j = i; j < i+10; ++j)
    {
      if (j >= keyframes.size()) continue;
      
      PointCloudT temp;
      pcl::transformPointCloud(keyframes[j].cloud, temp, eigenFromTf(keyframes[j].pose));
      *agr += temp;
    }
    
    double best_angle;
    
    printf("analyzing keyframe %d...\n", i);
    analyzeCloud(agr, best_angle);
    printf("analyzing keyframe %d done.\n", i);
    cv::waitKey(100);
    
    tf::Quaternion q;
    q.setRPY(0, 0, best_angle * M_PI / 180);
    tf::Transform tf;
    tf.setIdentity();
    tf.setRotation(q);
    
    for (unsigned int j = i; j < i+10; ++j)
    {
      if (j >= keyframes.size()) continue;

      PointCloudT temp;
      pcl::transformPointCloud(keyframes[j].cloud, temp, eigenFromTf(keyframes[j].pose));    
      pcl::transformPointCloud(temp, keyframes[j].cloud, eigenFromTf(tf));
    }
  }

  saveKeyframes(keyframes, "/home/idyanov/ros/images/ccny_3rooms_seq_loop_hist");
*/
}

bool analyzeCloud(
  const PointCloudT::Ptr& cloud,
  double& best_angle)
{
  // params
  double vgf_res = 0.01;
  double degrees_per_bin = 0.25;

  // filter cloud
  printf("Filtering cloud\n");
  pcl::VoxelGrid<PointT> vgf;
  PointCloudT::Ptr cloud_f;
  cloud_f.reset(new PointCloudT());
  vgf.setInputCloud(cloud);
  vgf.setLeafSize(vgf_res, vgf_res, vgf_res);
  vgf.filter(*cloud_f);
  
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  printf("Creating kd-tree\n");
  pcl::search::KdTree<PointT>::Ptr mls_tree;
  mls_tree.reset(new pcl::search::KdTree<PointT>());

  // smooth using mls
  printf("MLS\n");
  pcl::MovingLeastSquares<PointT, pcl::PointXYZRGBNormal> mls;
  mls.setInputCloud (cloud_f);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (mls_tree);
  mls.setSearchRadius (0.05);
  mls.reconstruct(*cloud_f); 

  // Compute the features
  printf("Estimating normals\n");

  pcl::search::KdTree<PointT>::Ptr tree;
  tree.reset(new pcl::search::KdTree<PointT>());

  pcl::NormalEstimation<PointT, pcl::PointXYZRGBNormal> ne;
  ne.setRadiusSearch(0.10);
  ne.setInputCloud(cloud_f);
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals;
  cloud_normals.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  ne.compute (*cloud_normals);

  // build histogram
  printf("building histogram\n");
  cv::Mat histogram;
  buildPhiHistogram(*cloud_normals, histogram, degrees_per_bin);

  // show histogram
  cv::Mat hist_img;
  createImageFromHistogram(histogram, hist_img);
  cv::imshow("Histogram Phi", hist_img);

  // find alignement
  cv::Mat hist_exp;
  buildExpectedPhiHistorgtam(hist_exp, degrees_per_bin, 2.0);
  cv::Mat hist_exp_img;
  createImageFromHistogram(hist_exp, hist_exp_img);
  cv::imshow("hist_exp_img", hist_exp_img);
  alignHistogram(histogram, hist_exp, degrees_per_bin, best_angle);

  if (best_angle > 45) best_angle-=90.0;
   
  return true;
}

bool analyzeKeyframe(
  RGBDKeyframe& keyframe,
  double& best_angle)
{
  // params
  double vgf_res = 0.01;
  double degrees_per_bin = 0.25;

  // rotate point cloud into global frame
  PointCloudT::Ptr cloud_tf;
  cloud_tf.reset(new PointCloudT());
  pcl::transformPointCloud(keyframe.cloud, *cloud_tf, eigenFromTf(keyframe.pose));

  // filter cloud
  printf("Filtering cloud\n");
  pcl::VoxelGrid<PointT> vgf;
  PointCloudT::Ptr cloud_f;
  cloud_f.reset(new PointCloudT());
  vgf.setInputCloud(cloud_tf);
  vgf.setLeafSize(vgf_res, vgf_res, vgf_res);
  vgf.filter(*cloud_f);

  printf("Cloud has %d points\n", (int)cloud_f->points.size());

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  printf("Creating kd-tree\n");
  pcl::search::KdTree<PointT>::Ptr mls_tree;
  mls_tree.reset(new pcl::search::KdTree<PointT>());

  // smooth using mls
  printf("MLS\n");
  pcl::MovingLeastSquares<PointT, pcl::PointXYZRGBNormal> mls;
  mls.setInputCloud (cloud_f);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (mls_tree);
  mls.setSearchRadius (0.05);
  mls.reconstruct(*cloud_f); 

  // Compute the features
  printf("Estimating normals\n");

  pcl::search::KdTree<PointT>::Ptr tree;
  tree.reset(new pcl::search::KdTree<PointT>());

  pcl::NormalEstimation<PointT, pcl::PointXYZRGBNormal> ne;
  ne.setRadiusSearch(0.10);
  ne.setInputCloud(cloud_f);
  ne.setSearchMethod(tree);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals;
  cloud_normals.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  ne.compute (*cloud_normals);

  // build histogram
  printf("building histogram\n");
  cv::Mat histogram;
  buildPhiHistogram(*cloud_normals, histogram, degrees_per_bin);

  // show histogram
  cv::Mat hist_img;
  createImageFromHistogram(histogram, hist_img);
  cv::imshow("Histogram Phi", hist_img);
  cv::imshow("RGB", keyframe.rgb_img);

  // find alignement
  cv::Mat hist_exp;
  buildExpectedPhiHistorgtam(hist_exp, degrees_per_bin, 2.0);
  cv::Mat hist_exp_img;
  createImageFromHistogram(hist_exp, hist_exp_img);
  cv::imshow("hist_exp_img", hist_exp_img);
  cv::waitKey(0);
  
  alignHistogram(histogram, hist_exp, degrees_per_bin, best_angle);

  if (best_angle > 45) best_angle-=90.0;
  
  /*
  tf::Quaternion q;
  q.setRPY(0, 0, best_angle * M_PI / 180);
  tf::Transform tf;
  tf.setIdentity();
  tf.setRotation(q);
  
  pcl::transformPointCloud(*cloud_f, *cloud_f, eigenFromTf(tf));
  keyframe.cloud = *cloud_f;
  */
  
  // TODO: return false if bad histogram
  return true;
}
  
} // namespace ccny_rgbd
