#ifndef CCNY_RGBD_TEST_PROJECTION_H
#define CCNY_RGBD_TEST_PROJECTION_H

#include "ccny_rgbd/rgbd_util.h"
#include "ccny_rgbd/structures/rgbd_frame.h"

namespace ccny_rgbd {

void test_projection(int id);

void paintPixel(
  double v, double u, double z, const cv::Vec3f& color_rgb,
  cv::Mat& rgb_sum_img, cv::Mat& depth_img, cv::Mat& weight_img)
{
  double s = 3; 
  double w = s / z;
  
  // depth in mm
  uint16_t depth = z * 1000.0;
  
  int uu_min = std::max((int)(u - w), 0);
  int uu_max = std::min((int)(u + w), rgb_sum_img.cols - 1);
  
  int vv_min = std::max((int)(v - w), 0);
  int vv_max = std::min((int)(v + w), rgb_sum_img.rows - 1);
  
  for (int vv = vv_min; vv <= vv_max; ++vv)
  for (int uu = uu_min; uu <= uu_max; ++uu)
  {
    uint16_t& depth_out = depth_img.at<uint16_t>(vv,uu);
    cv::Vec3f& rgb_sum_out = rgb_sum_img.at<cv::Vec3f>(vv,uu);  
    float& weight_out = weight_img.at<float>(vv,uu);  
    
    // empty buffer - add color
    if (depth_out == 0)
    {
      rgb_sum_out = color_rgb;           
      depth_out = depth; 
      weight_out = 1.0;
    }
    // closer than current z
    else if (depth < depth_out)
    {     
      double depth_diff = std::abs(depth - depth_out);
      
      if (depth_diff > 30) //mm
      {
        depth_out = depth;
        rgb_sum_out = color_rgb;
        weight_out = 1.0;
      }
      else
      {
        depth_out = depth;
        rgb_sum_out += color_rgb;
        weight_out += 1.0;
      }
    }  
  }
}

void projectCloudToVirtualImage(
  const PointCloudT& cloud,
  const Matrix3f& rmat,
  const Vector3f& tvec,
  const Matrix3f& intrinsic,
  int width, int height,
  cv::Mat& rgb_img,
  cv::Mat& depth_img)
{
  rgb_img   = cv::Mat::zeros(height, width, CV_8UC3);
  depth_img = cv::Mat::zeros(height, width, CV_16UC1);

  cv::Mat rgb_sum_img = cv::Mat::zeros(height, width, CV_32FC3);
  cv::Mat weight_img  = cv::Mat::zeros(height, width, CV_32FC1);
 
  for (uint i=0; i<cloud.points.size(); ++i)
  {
    // convert from pcl PointT to Eigen Vector3f
    PointT point = cloud.points[i];
    Vector3f p_world;
    if (isnan(point.x) || isnan(point.y) || isnan(point.z) ) continue;

    p_world(0,0) = point.x;
    p_world(1,0) = point.y;
    p_world(2,0) = point.z;
    
    // transforms into the camera frame  
    Vector3f p_cam = rmat * p_world + tvec; 
    double z = p_cam(2,0);
    
    // skip behind camera
    if (z <= 0.0) continue;

    //projection into the imiage plane   
    Vector3f p_proj = intrinsic * p_cam;                    

    double u = p_proj(0,0) / p_proj(2,0);
    double v = p_proj(1,0) / p_proj(2,0);
    
    //takes only the visible points  
    if ((u<width) && (u>=0) && (v<height) && (v>=0)) 
    {
      cv::Vec3f color_rgb;
      color_rgb[0] = point.b;  
      color_rgb[1] = point.g;
      color_rgb[2] = point.r;

      paintPixel(v, u, z, color_rgb, rgb_sum_img, depth_img, weight_img);
    }
  }   
  
  // normalize
  for (int v = 0; v <= rgb_img.rows; ++v)
  for (int u = 0; u <= rgb_img.cols; ++u)
  {
    uint16_t& depth = depth_img.at<uint16_t>(v,u);
    cv::Vec3b& rgb = rgb_img.at<cv::Vec3b>(v,u);  
    
    cv::Vec3f rgb_sum = rgb_sum_img.at<cv::Vec3f>(v,u);  
    float weight = weight_img.at<float>(v,u);   
    
    if (weight != 0) 
    {     
      rgb[0] = rgb_sum[0] / weight;
      rgb[1] = rgb_sum[1] / weight;
      rgb[2] = rgb_sum[2] / weight;
    }
  }
}



}//namespace ccny_rgbd

#endif // CCNY_RGBD_TEST_PROJECTION_H
