#include "ccny_rgbd/rgbd_normals.h"

#include <pcl/surface/mls.h>
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

int main(int argc, char** argv)
{
  ccny_rgbd::analyzeKeyframes();
  return 0;
}

namespace ccny_rgbd {

void analyzeKeyframes()
{
  KeyframeVector keyframes;
  loadKeyframes(keyframes, "/home/idyanov/ros/images/ccny_2rooms_seq_loop");

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

  saveKeyframes(keyframes, "/home/idyanov/ros/images/ccny_2rooms_seq_loop_hist");

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

void analyzeKeyframe(RGBDKeyframe& keyframe)
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
  double best_angle;
  alignHistogram(histogram, hist_exp, degrees_per_bin, best_angle);

  if (best_angle > 45) best_angle-=90.0;
  tf::Quaternion q;
  q.setRPY(0, 0, best_angle * M_PI / 180);
  tf::Transform tf;
  tf.setIdentity();
  tf.setRotation(q);
  
  pcl::transformPointCloud(*cloud_f, *cloud_f, eigenFromTf(tf));
  keyframe.cloud = *cloud_f;

  cv::waitKey(0);
}

void buildGlobalMap(
  const KeyframeVector& keyframes,
  PointCloudT& global_map,
  double resolution)
{
  PointCloudT::Ptr map_unf(new PointCloudT());

  // create global cloud
  for (unsigned int i = 0; i < keyframes.size(); ++i)
  {
    printf("Adding in keyframe %d of %d\n", i, (int)keyframes.size());
    const RGBDKeyframe& keyframe = keyframes[i];
   
    PointCloudT cloud; 
    pcl::transformPointCloud(keyframe.cloud, cloud, eigenFromTf(keyframe.pose));

    *map_unf += cloud;
  }

  // filter cloud
  printf("Filtering cloud\n");
  PointCloudT::Ptr cloud_f;
  cloud_f.reset(new PointCloudT());
  pcl::VoxelGrid<PointT> vgf;
  vgf.setInputCloud(map_unf);
  vgf.setLeafSize(resolution, resolution, resolution);
  vgf.filter(global_map);
}

void filterCloudByHeight(
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_in,
  pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud_out,
  double min_z,
  double max_z)
{
  for (unsigned int i = 0; i < cloud_in.points.size(); ++i)
  {
    const pcl::PointXYZRGBNormal& p = cloud_in.points[i]; 
    
    if (p.z >= min_z && p.z < max_z)
      cloud_out.push_back(p); 
  }
}

bool alignHistogram(
  const cv::Mat& hist,
  const cv::Mat& hist_exp,
  double hist_resolution,
  double& best_angle)
{
  // check diff
  int best_i = 0;
  double best_diff = 9999999999;

  for (int i = 0; i < 90.0 / hist_resolution; ++i)
  {
    cv::Mat hist_shifted;
    shiftHistogram(hist, hist_shifted, i);

    double diff = cv::compareHist(hist_shifted, hist_exp, CV_COMP_BHATTACHARYYA);
    if (std::abs(diff) < best_diff)
    {
      best_diff = std::abs(diff);
      best_i = i;
    }
  }

  best_angle = best_i * hist_resolution;

  printf("BEST ANGLE: %f\n", best_angle);

  cv::Mat hist_best;
  shiftHistogram(hist, hist_best, best_i);
  cv::Mat hist_best_img;
  createImageFromHistogram(hist_best, hist_best_img);
  cv::imshow("hist_best_img", hist_best_img);

  return true;
}

void g()
{
  double vgf_res = 0.01;

  // read in
  printf("Reading cloud\n");
  PointCloudT::Ptr cloud;
  cloud.reset(new PointCloudT());
  pcl::PCDReader reader;
  reader.read ("/home/idryanov/ros/office_loop.pcd", *cloud);

  // filter cloud
  printf("Filtering cloud\n");
  PointCloudT::Ptr cloud_f;
  cloud_f.reset(new PointCloudT());
  pcl::VoxelGrid<PointT> vgf;
  vgf.setInputCloud(cloud);
  vgf.setLeafSize(vgf_res, vgf_res, vgf_res);
  vgf.filter(*cloud_f);
 
  // rotate 45 deg
  tf::Transform t;
  t.setOrigin(tf::Vector3(0,0,0));
  tf::Quaternion q;
  q.setRPY(0, 0, M_PI/6.0);
  t.setRotation(q);
  pcl::transformPointCloud(*cloud_f, *cloud_f, eigenFromTf(t)); 

  // show map
  cv::Mat map_r;
  create2DProjectionImage(*cloud_f, map_r);
  cv::imshow("map_r", map_r);
  cv::waitKey(0);

  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<PointT, pcl::PointXYZRGBNormal> ne;
  ne.setInputCloud(cloud_f);

  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
  printf("Creating kd-tree\n");
  pcl::search::KdTree<PointT>::Ptr tree;
  tree.reset(new pcl::search::KdTree<PointT>());
  ne.setSearchMethod(tree);

  // Output datasets
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals;
  cloud_normals.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  // Use all neighbors in a sphere of radius x cm
  ne.setRadiusSearch(0.05);

  // Compute the features
  printf("Estimating normals\n");
  ne.compute (*cloud_normals);

  for (unsigned int i = 0; i < cloud_f->points.size(); ++i)
  {
    const PointT& p_in = cloud_f->points[i]; 
    pcl::PointXYZRGBNormal& p_out = cloud_normals->points[i]; 

    p_out.x   = p_in.x;
    p_out.y   = p_in.y;
    p_out.z   = p_in.z;
    p_out.rgb = p_in.rgb;
  }

  // write out  
  printf("Writing out\n");
  pcl::PCDWriter writer;
  writer.write ("/home/idryanov/cloud_00_n.pcd", *cloud_normals);

  // create expected histogram
  double hist_resolution = 0.25;
  cv::Mat hist_exp;
  buildExpectedPhiHistorgtam(hist_exp, hist_resolution, 5.0); 
  cv::Mat hist_exp_img;
  createImageFromHistogram(hist_exp, hist_exp_img);
  cv::imshow("hist_exp_img", hist_exp_img);
  cv::waitKey(0);

  // create histogram
  printf("creating histogram\n");
  cv::Mat hist;
  buildPhiHistogram(*cloud_normals, hist, hist_resolution);

  // show histogram
  printf("showing histogram\n");
  cv::Mat hist_img;
  createImageFromHistogram(hist, hist_img);
  cv::imshow("Histogram Phi", hist_img);
  cv::waitKey(0);

  // check diff
  int best_i = 0;
  double best_diff = 9999999999;

  for (int i = 0; i < 90.0 / hist_resolution; ++i)
  {
    cv::Mat hist_shifted;
    shiftHistogram(hist, hist_shifted, i);

    cv::Mat hist_shifted_img;
    createImageFromHistogram(hist_shifted, hist_shifted_img);
    cv::imshow("hist_shifted_img", hist_shifted_img);
    cv::waitKey(10);

    double diff = cv::compareHist(hist_shifted, hist_exp, CV_COMP_BHATTACHARYYA);
    if (std::abs(diff) < best_diff)
    {
      best_diff = std::abs(diff);
      best_i = i;
    }
  }

  double best_angle = best_i * hist_resolution;
  printf("best_angle: %f\n", best_angle);
  
  cv::Mat hist_best;
  shiftHistogram(hist, hist_best, best_i);
  cv::Mat hist_best_img;
  createImageFromHistogram(hist_best, hist_best_img);
  cv::imshow("hist_best_img", hist_best_img);
  cv::waitKey(0);

  // find histogram maximum
  printf("finding maximum\n");
  double min_val, max_val;
  cv::Point minLoc, maxLoc;
  cv::minMaxLoc(hist, &min_val, &max_val, &minLoc, &maxLoc);
  double max_angle = maxLoc.x * hist_resolution;
  printf("max_angle: %f\n", max_angle);

  // derotate
  tf::Transform t1;
  t1.setOrigin(tf::Vector3(0,0,0));
  tf::Quaternion q1;
  q1.setRPY(0, 0, best_angle * M_PI/180.0);
  t1.setRotation(q1);
  pcl::transformPointCloud(*cloud_f, *cloud_f, eigenFromTf(t1)); 

  // show map
  cv::Mat map_f;
  create2DProjectionImage(*cloud_f, map_f);
  cv::imshow("map_f", map_f);
  cv::waitKey(0);

  printf("Done\n");
}

void normalizeHistogram(cv::Mat& histogram)
{
  float sum = 0;

  for (unsigned int u = 0; u < histogram.cols; ++u)
    sum += histogram.at<float>(0,u);

  histogram = histogram / sum;
}

void shiftHistogram(
  const cv::Mat& hist_in,
  cv::Mat& hist_out,
  int bins)
{
  hist_out = cv::Mat(hist_in.size(), CV_32FC1);
  int w = hist_in.cols;
  for (int u = 0; u < w; ++u)
  {
    int u_shifted = (u + bins) % w;

    hist_out.at<float>(0, u_shifted) = hist_in.at<float>(0, u);
  } 
}

void buildExpectedPhiHistorgtam(
  cv::Mat& histogram,
  double degrees_per_bin,
  double stdev)
{
  int n_bins = (int)(360.0 / degrees_per_bin);
  histogram = cv::Mat::zeros(1, n_bins, CV_32FC1);

  double s = stdev  / degrees_per_bin;

  double mean[4];
  mean[0] =   0.0 / degrees_per_bin;
  mean[1] =  90.0 / degrees_per_bin;
  mean[2] = 180.0 / degrees_per_bin;
  mean[3] = 270.0 / degrees_per_bin;

  double a = 1.0 / (s * sqrt(2.0 * M_PI));
  double b = 2.0 * s * s; 

  for (int u = 0; u < n_bins; ++u)
  {
    float& bin = histogram.at<float>(0, u);

    // accumulate 4 gaussians
    for (int g = 0; g < 4; g++)
    {
      int x = u - mean[g];
  
      // wrap around to closer distance
      if (x < -n_bins/2) x += n_bins;
      if (x >= n_bins/2) x -= n_bins;

      float r = a * exp(-x*x / b);

      bin += r;
    }
  }

  normalizeHistogram(histogram);
}  

void buildPhiHistogram(
  const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud,
  cv::Mat& histogram,
  double degrees_per_bin)
{
  int phi_bins = (int)(360.0 / degrees_per_bin);
  histogram = cv::Mat::zeros(1, phi_bins, CV_32FC1);
  
  for (unsigned int i = 0; i < cloud.points.size(); ++i)
  {
    const pcl::PointXYZRGBNormal& p = cloud.points[i]; 

    double nx = p.normal_x;
    double ny = p.normal_y;
    double nz = p.normal_z;

    if (isnan(nx) || isnan(ny) || isnan(nz)) continue;

    double r = sqrt(nx*nx + ny*ny + nz*nz);
    double theta = acos(nz/r);
    double phi   = atan2(ny, nx);

    double phi_deg   = phi   * 180.0 / M_PI;
    double theta_deg = theta * 180.0 / M_PI; 

    // normalize phi to [0, 360)
    if (phi_deg < 0.0) phi_deg += 360.0;

    // only consider points which are close to vertical
    if (std::abs(90-theta_deg) > 3.0) continue; 

    int idx_phi = (int)(phi_deg / degrees_per_bin);

    float& bin = histogram.at<float>(0, idx_phi);
    bin = bin + 1.0; 
  }

  normalizeHistogram(histogram);
}

void create8bitHistogram(
  const cv::Mat& histogram,
  cv::Mat& histogram_norm)
{
  // find max value
  double min_val, max_val;
  cv::Point minLoc, maxLoc;
  cv::minMaxLoc(histogram, &min_val, &max_val, &minLoc, &maxLoc);

  // rescale so that max = 255, for visualization purposes
  cv::Mat temp = histogram.clone();
  temp = temp * 255.0 / max_val;

  // convert to uint
  histogram_norm = cv::Mat::zeros(histogram.size(), CV_8UC1); 
  temp.convertTo(histogram_norm, CV_8UC1); 
}

void createImageFromHistogram(
  const cv::Mat& histogram,
  cv::Mat& image)
{
  // normalize the histogram in range 0 - 255
  cv::Mat hist_norm;
  create8bitHistogram(histogram, hist_norm);

  image = cv::Mat::zeros(256, histogram.cols, CV_8UC1);
  for (int u = 0; u < histogram.cols; ++u)
  {
    uint8_t val = hist_norm.at<uint8_t>(0, u);
    for (int v = 0; v < val; ++v)
      image.at<uint8_t>(255-v, u) = 255;
  }
}

void create2DProjectionImage(
  const PointCloudT& cloud, 
  cv::Mat& img,
  double min_z,
  double max_z)
{
  double resolution = 0.02; // 2cm
  double w = 10.0;
  double h = 10.0;
  double cx = 5.0;
  double cy = 5.0;
  
  img = cv::Mat::zeros(h/resolution, w/resolution, CV_8UC1);

  for (unsigned int i = 0; i < cloud.points.size(); ++i)
  {
    const PointT& p = cloud.points[i];

    // filter z
    if (isnan(p.z) || p.z >= max_z || p.z < min_z) continue;

    int u = (p.x + cx)/resolution;
    int v = (p.y + cy)/resolution;
    
    //printf("%d, %d\n", u, v);

    uint8_t& bin = img.at<uint8_t>(v, u);
    if(bin < 255) bin++;
  }
}

void f(const RGBDFrame& frame)
{
  // build point cloud
  PointCloudT::Ptr cloud;
  cloud.reset(new PointCloudT());
  buildPointCloud(frame.depth_img, frame.rgb_img, 
    frame.model.fullIntrinsicMatrix(),
    *cloud);

  // write out
  pcl::PCDWriter writer;
  writer.write ("/home/idryanov/cloud_00.pcd", *cloud);
}

void buildPointCloud(
  const cv::Mat& depth_img_rect_reg,
  const cv::Mat& rgb_img_rect,
  const cv::Mat& intr_rect_rgb,
  PointCloudT& cloud)
{
  int w = rgb_img_rect.cols;
  int h = rgb_img_rect.rows;
  
  double cx = intr_rect_rgb.at<double>(0,2);
  double cy = intr_rect_rgb.at<double>(1,2);
  double fx_inv = 1.0 / intr_rect_rgb.at<double>(0,0);
  double fy_inv = 1.0 / intr_rect_rgb.at<double>(1,1);

  cloud.resize(w*h);
  
  for (int u = 0; u < w; ++u)
  for (int v = 0; v < h; ++v)
  {
    uint16_t z = depth_img_rect_reg.at<uint16_t>(v, u);
    const cv::Vec3b& c = rgb_img_rect.at<cv::Vec3b>(v, u);
    
    PointT& pt = cloud.points[v*w + u];
    
    if (z != 0)
    {  
      double z_metric = z * 0.001;
             
      pt.x = z_metric * ((u - cx) * fx_inv);
      pt.y = z_metric * ((v - cy) * fy_inv);
      pt.z = z_metric;
  
      pt.r = c[2];
      pt.g = c[1];
      pt.b = c[0];
    }
    else
    {
      pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
    }
  }  
  
  cloud.width = w;
  cloud.height = h;
  cloud.is_dense = true;
}
} // namespace ccny_rgbd
