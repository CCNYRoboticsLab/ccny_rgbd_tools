/**
 *  @file rgbd_util.cpp
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2013, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd {

void removeInvalidMeans(
  const Vector3fVector& means,
  const BoolVector& valid,
  Vector3fVector& means_f)
{
  unsigned int size = valid.size(); 
  for(unsigned int i = 0; i < size; ++i)
  {
    if (valid[i])
    {
      const Vector3f& mean = means[i];
      means_f.push_back(mean);
    }
  }
}

void removeInvalidDistributions(
  const Vector3fVector& means,
  const Matrix3fVector& covariances,
  const BoolVector& valid,
  Vector3fVector& means_f,
  Matrix3fVector& covariances_f)
{
  unsigned int size = valid.size(); 
  for(unsigned int i = 0; i < size; ++i)
  {
    if (valid[i])
    {
      const Vector3f& mean = means[i];
      const Matrix3f& cov  = covariances[i];

      means_f.push_back(mean);
      covariances_f.push_back(cov);
    }
  }
}

void pointCloudFromMeans(
  const Vector3fVector& means,
  PointCloudFeature& cloud)
{
  unsigned int size = means.size(); 
  cloud.points.resize(size);
  for(unsigned int i = 0; i < size; ++i)
  {
    const Vector3f& m = means[i];
    PointFeature& p = cloud.points[i];

    p.x = m(0,0);
    p.y = m(1,0);
    p.z = m(2,0);
  }
  
  cloud.height = 1;
  cloud.width = size;
  cloud.is_dense = true;
}


void buildPointCloud(
  const cv::Mat& depth_img_rect,
  const cv::Mat& intr_rect_ir,
  PointCloudT& cloud)
{
  int w = depth_img_rect.cols;
  int h = depth_img_rect.rows;
  
  double cx = intr_rect_ir.at<double>(0,2);
  double cy = intr_rect_ir.at<double>(1,2);
  double fx_inv = 1.0 / intr_rect_ir.at<double>(0,0);
  double fy_inv = 1.0 / intr_rect_ir.at<double>(1,1);

  cloud.resize(w*h);

  for (int u = 0; u < w; ++u)
  for (int v = 0; v < h; ++v)
  {
    uint16_t z = depth_img_rect.at<uint16_t>(v, u);   
    PointT& pt = cloud.points[v*w + u];
    
    if (z != 0)
    {  
      double z_metric = z * 0.001;
             
      pt.x = z_metric * ((u - cx) * fx_inv);
      pt.y = z_metric * ((v - cy) * fy_inv);
      pt.z = z_metric;  
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

void buildRegisteredDepthImage(
  const cv::Mat& intr_rect_ir,
  const cv::Mat& intr_rect_rgb,
  const cv::Mat& ir2rgb,
  const cv::Mat& depth_img_rect,
        cv::Mat& depth_img_rect_reg)
{  
  int w = depth_img_rect.cols;
  int h = depth_img_rect.rows;
      
  depth_img_rect_reg = cv::Mat::zeros(h, w, CV_16UC1); 
  
  cv::Mat intr_rect_ir_inv = intr_rect_ir.inv();
  
  // Eigen intr_rect_rgb (3x3)
  Eigen::Matrix<double, 3, 3> intr_rect_rgb_eigen;  
  for (int u = 0; u < 3; ++u)
  for (int v = 0; v < 3; ++v)
    intr_rect_rgb_eigen(v,u) =  intr_rect_rgb.at<double>(v, u); 
  
  // Eigen rgb2ir_eigen (3x4)
  Eigen::Matrix<double, 3, 4> ir2rgb_eigen;
  for (int u = 0; u < 4; ++u)
  for (int v = 0; v < 3; ++v)
    ir2rgb_eigen(v,u) =  ir2rgb.at<double>(v, u);
   
  // Eigen intr_rect_ir_inv (4x4)
  Eigen::Matrix4d intr_rect_ir_inv_eigen;  
  for (int v = 0; v < 3; ++v)
  for (int u = 0; u < 3; ++u)
    intr_rect_ir_inv_eigen(v,u) = intr_rect_ir_inv.at<double>(v,u);
  
  intr_rect_ir_inv_eigen(0, 3) = 0;
  intr_rect_ir_inv_eigen(1, 3) = 0;
  intr_rect_ir_inv_eigen(2, 3) = 0;
  intr_rect_ir_inv_eigen(3, 0) = 0;
  intr_rect_ir_inv_eigen(3, 1) = 0;
  intr_rect_ir_inv_eigen(3, 2) = 0;
  intr_rect_ir_inv_eigen(3, 3) = 1;
    
  // multiply into single (3x4) matrix
  Eigen::Matrix<double, 3, 4> H_eigen = 
    intr_rect_rgb_eigen * (ir2rgb_eigen * intr_rect_ir_inv_eigen);

  // *** reproject  
  
  Eigen::Vector3d p_rgb;
  Eigen::Vector4d p_depth;
  
  for (int v = 0; v < h; ++v)
  for (int u = 0; u < w; ++u)
  {
    uint16_t z = depth_img_rect.at<uint16_t>(v,u);
    
    if (z != 0)
    {    
      p_depth(0,0) = u * z;
      p_depth(1,0) = v * z;
      p_depth(2,0) = z;
      p_depth(3,0) = 1.0; 
      p_rgb = H_eigen * p_depth;
         
      double px = p_rgb(0,0);
      double py = p_rgb(1,0);
      double pz = p_rgb(2,0);
            
      int qu = (int)(px / pz);
      int qv = (int)(py / pz);  
        
      // skip outside of image 
      if (qu < 0 || qu >= w || qv < 0 || qv >= h) continue;
    
      uint16_t& val = depth_img_rect_reg.at<uint16_t>(qv, qu);
    
      // z buffering
      if (val == 0 || val > pz) val = pz;
    }
  }
}

void depthImageFloatTo16bit(
  const cv::Mat& depth_image_in,
  cv::Mat& depth_image_out)
{
  depth_image_in.convertTo(depth_image_out, CV_16UC1, 1000.0);
}

void eigenAffineToOpenCVRt(
  const AffineTransform& transform,
  cv::Mat& R,
  cv::Mat& t)
{
  // extract translation
  t = cv::Mat(3, 1, CV_64F);
  t.at<double>(0,0) = transform(0,3);
  t.at<double>(1,0) = transform(1,3);
  t.at<double>(2,0) = transform(2,3);

  // extract rotation
  R = cv::Mat(3, 3, CV_64F);
  for(int i = 0; i < 3; ++i)
  for(int j = 0; j < 3; ++j)     
    R.at<double>(j,i) = transform(j,i);
}

void openCVRtToEigenAffine(
  const cv::Mat& R,
  const cv::Mat& t,
  AffineTransform& transform)
{
  // extract translation
  transform(0,3) = t.at<double>(0,0);
  transform(1,3) = t.at<double>(1,0);
  transform(2,3) = t.at<double>(2,0);
  
  // extract rotation
  for(int i = 0; i < 3; ++i)
  for(int j = 0; j < 3; ++j)     
    transform(j,i) = R.at<double>(j,i);

  // last row
  transform(3,0) = 0.0;
  transform(3,1) = 0.0;
  transform(3,2) = 0.0;
  transform(3,3) = 1.0;
}
    
void eigenAffineToXYZRPY(
  const AffineTransform& transform, 
  float& x, float& y, float& z, 
  float& roll, float& pitch, float& yaw)
{
   x = transform(0,3);
   y = transform(1,3);
   z = transform(2,3);
   
   roll  = atan2f(transform(2,1), transform(2,2));
   pitch = asinf(-transform(2,0));
   yaw   = atan2f(transform(1,0), transform(0,0));
}
 
void XYZRPYToEigenAffine(
  float x, float y, float z, 
  float roll, float pitch, float yaw, 
  AffineTransform& t)
{
  float A=cosf(yaw),  B=sinf(yaw),  C=cosf(pitch), D=sinf(pitch),
  E=cosf(roll), F=sinf(roll), DE=D*E,        DF=D*F;
  t(0,0) = A*C;  t(0,1) = A*DF - B*E;  t(0,2) = B*F + A*DE;  t(0,3) = x;
  t(1,0) = B*C;  t(1,1) = A*E + B*DF;  t(1,2) = B*DE - A*F;  t(1,3) = y;
  t(2,0) = -D;   t(2,1) = C*F;         t(2,2) = C*E;         t(2,3) = z;
  t(3,0) = 0;    t(3,1) = 0;           t(3,2) = 0;           t(3,3) = 1;
}
  
void transformDistributions(
  Vector3fVector& means,
  Matrix3fVector& covariances,
  const AffineTransform& transform)
{
  Matrix3f R = transform.rotation();
  Vector3f t = transform.translation();
  Matrix3f R_T = R.transpose();
  
  unsigned int size = means.size(); 
  for(unsigned int i = 0; i < size; ++i)
  {
    Vector3f& m = means[i];
    Matrix3f& c = covariances[i];
    m = R * m + t;
    c = R * c * R_T;
  }
}

void getTfDifference(
  const AffineTransform& transform, 
  double& dist, double& angle)
{
  Matrix3f R = transform.rotation();
  Vector3f t = transform.translation();

  dist = sqrt(t(0,0)*t(0,0) + t(1,0)*t(1,0) + t(2,0)*t(2,0));
    
  double trace = R(0,0) + R(1,1) + R(2,2);
  angle = acos(std::min(1.0, std::max(-1.0, (trace - 1.0)/2.0)));
}
 
} // namespace ccny_rgbd
