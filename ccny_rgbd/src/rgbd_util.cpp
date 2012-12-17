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

void getTfDifference(const tf::Transform& motion, double& dist, double& angle)
{
  dist = motion.getOrigin().length();
  double trace = motion.getBasis()[0][0] + motion.getBasis()[1][1] + motion.getBasis()[2][2];
  angle = acos(std::min(1.0, std::max(-1.0, (trace - 1.0)/2.0)));
}

void getTfDifference(const tf::Transform& a, const tf::Transform b, double& dist, double& angle)
{
  tf::Transform motion = a.inverse() * b;
  getTfDifference(motion, dist, angle);
}

tf::Transform tfFromEigen(Eigen::Matrix4f trans)
{
  tf::Matrix3x3 btm;
  btm.setValue(trans(0,0),trans(0,1),trans(0,2),
            trans(1,0),trans(1,1),trans(1,2),
            trans(2,0),trans(2,1),trans(2,2));
  tf::Transform ret;
  ret.setOrigin(tf::Vector3(trans(0,3),trans(1,3),trans(2,3)));
  ret.setBasis(btm);
  return ret;
}

Eigen::Matrix4f eigenFromTf(const tf::Transform& tf)
{
   Eigen::Matrix4f out_mat;

   double mv[12];
   tf.getBasis().getOpenGLSubMatrix(mv);

   tf::Vector3 origin = tf.getOrigin();

   out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
   out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
   out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];

   out_mat (3, 0) = out_mat (3, 1) = out_mat (3, 2) = 0; out_mat (3, 3) = 1;
   out_mat (0, 3) = origin.x ();
   out_mat (1, 3) = origin.y ();
   out_mat (2, 3) = origin.z ();

   return out_mat;
}

void tfToXYZRPY(
  const tf::Transform& t,
  double& x,    double& y,     double& z,
  double& roll, double& pitch, double& yaw)
{
  x = t.getOrigin().getX();
  y = t.getOrigin().getY();
  z = t.getOrigin().getZ();

  tf::Matrix3x3  m(t.getRotation());
  m.getRPY(roll, pitch, yaw);
}

bool tfGreaterThan(const tf::Transform& tf, double dist, double angle)
{
  double d = tf.getOrigin().length();
  
  if (d >= dist) return true;

  double trace = tf.getBasis()[0][0] + tf.getBasis()[1][1] + tf.getBasis()[2][2];
  double a = acos(std::min(1.0, std::max(-1.0, (trace - 1.0)/2.0)));

  if (a > angle) return true;
  
  return false;
}

double getMsDuration(const ros::WallTime& start)
{
  return (ros::WallTime::now() - start).toSec() * 1000.0;
}

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

void tfToEigenRt(
  const tf::Transform& tf, 
  Matrix3f& R, 
  Vector3f& t)
{
   double mv[12];
   tf.getBasis().getOpenGLSubMatrix(mv);

   tf::Vector3 origin = tf.getOrigin();

   R(0, 0) = mv[0]; R(0, 1) = mv[4]; R(0, 2) = mv[8];
   R(1, 0) = mv[1]; R(1, 1) = mv[5]; R(1, 2) = mv[9];
   R(2, 0) = mv[2]; R(2, 1) = mv[6]; R(2, 2) = mv[10];

   t(0, 0) = origin.x();
   t(1, 0) = origin.y();
   t(2, 0) = origin.z();
}

void tfToOpenCVRt(
  const tf::Transform& transform,
  cv::Mat& R,
  cv::Mat& t)
{
  // extract translation
  tf::Vector3 translation_tf = transform.getOrigin();
  t = cv::Mat(3, 1, CV_64F);
  t.at<double>(0,0) = translation_tf.getX();
  t.at<double>(1,0) = translation_tf.getY();
  t.at<double>(2,0) = translation_tf.getZ();

  // extract rotation
  tf::Matrix3x3 rotation_tf(transform.getRotation());
  R = cv::Mat(3, 3, CV_64F);
  for(int i = 0; i < 3; ++i)
  for(int j = 0; j < 3; ++j)     
    R.at<double>(j,i) = rotation_tf[j][i];
}

void openCVRToEigenR(
  const cv::Mat& R,
  Matrix3f& R_eigen)
{
  for(int i = 0; i < 3; ++i)
  for(int j = 0; j < 3; ++j)
    R_eigen(j,i) =  R.at<double>(j,i); 
}

void openCVRtToTf(
  const cv::Mat& R,
  const cv::Mat& t,
  tf::Transform& transform)
{
  tf::Vector3 translation_tf(
    t.at<double>(0,0),
    t.at<double>(1,0),
    t.at<double>(2,0));

  tf::Matrix3x3 rotation_tf;
  for(int i = 0; i < 3; ++i)
  for(int j = 0; j < 3; ++j)     
    rotation_tf[j][i] = R.at<double>(j,i);

  transform.setOrigin(translation_tf);
  transform.setBasis(rotation_tf);
}

void convertCameraInfoToMats(
  const CameraInfoMsg::ConstPtr camera_info_msg,
  cv::Mat& intr,
  cv::Mat& dist)
{
  // set intrinsic matrix from K vector
  intr = cv::Mat(3, 3, CV_64FC1);
  for (int idx = 0; idx < 9; ++idx)
  {
    int i = idx % 3;
    int j = idx / 3;
    intr.at<double>(j, i) = camera_info_msg->K[idx];
  }
  
  // set distortion matrix from D vector
  int d_size = camera_info_msg->D.size();
  dist = cv::Mat(1, d_size, CV_64FC1);
  for (int idx = 0; idx < d_size; ++idx)
  {
    dist.at<double>(0, idx) = camera_info_msg->D[idx];   
  }
}

void convertMatToCameraInfo(
  const cv::Mat& intr,
  CameraInfoMsg& camera_info_msg)
{
  // set D matrix to 0
  camera_info_msg.D.resize(5);
  std::fill(camera_info_msg.D.begin(), camera_info_msg.D.end(), 0.0);
  
  // set K matrix to optimal new camera matrix
  for (int i = 0; i < 3; ++i)
  for (int j = 0; j < 3; ++j)
    camera_info_msg.K[j*3 + i] = intr.at<double>(j,i);
  
  // set R matrix to identity
  std::fill(camera_info_msg.R.begin(), camera_info_msg.R.end(), 0.0);  
  camera_info_msg.R[0*3 + 0] = 1.0;
  camera_info_msg.R[1*3 + 1] = 1.0;
  camera_info_msg.R[2*3 + 2] = 1.0;
    
  //set P matrix to K
  std::fill(camera_info_msg.P.begin(), camera_info_msg.P.end(), 0.0);  
  for (int i = 0; i < 3; ++i)
  for (int j = 0; j < 3; ++j)
    camera_info_msg.P[j*4 + i] = intr.at<double>(j,i);
}

void transformMeans(
  Vector3fVector& means,
  const tf::Transform& transform)
{
  Matrix3f R;
  Vector3f t;
  tfToEigenRt(transform, R, t);
  
  unsigned int size = means.size(); 
  for(unsigned int i = 0; i < size; ++i)
  {
    Vector3f& m = means[i];
    m = R * m + t;
  }  
}

void transformDistributions(
  Vector3fVector& means,
  Matrix3fVector& covariances,
  const tf::Transform& transform)
{
  Matrix3f R;
  Vector3f t;
  tfToEigenRt(transform, R, t);
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

void projectCloudToImage(const PointCloudT& cloud,
                         const Matrix3f& rmat,
                         const Vector3f& tvec,
                         const Matrix3f& intrinsic,
                         uint width,
                         uint height,
                         cv::Mat& rgb_img,
                         cv::Mat& depth_img)
{
  
  rgb_img   = cv::Mat::zeros(height, width, CV_8UC3);
  depth_img = cv::Mat::zeros(height, width, CV_16UC1);

  for (uint i=0; i<cloud.points.size(); ++i)
  {
    // convert from pcl PointT to Eigen Vector3f
    PointT point = cloud.points[i];
    Vector3f p_world;
    p_world(0,0) = point.x;
    p_world(1,0) = point.y;
    p_world(2,0) = point.z;
    
    // transforms into the camera frame  
    Vector3f p_cam = rmat * p_world + tvec; 
    double depth = p_cam(2,0) * 1000.0;       //depth in millimiter
    
    if (depth <= 0) continue;

    //projection into the imiage plane   
    Vector3f p_proj = intrinsic * p_cam;                    
    double z_proj = p_proj(2,0);

    int u = (p_proj(0,0))/z_proj;
    int v = (p_proj(1,0))/z_proj;
    
    //takes only the visible points  
    if ((u<width) && (u>=0) && (v<height) && (v>=0)) 
    {
      cv::Vec3b color_rgb;
      color_rgb[0] = point.b;  
      color_rgb[1] = point.g;
      color_rgb[2] = point.r;
          
      if (depth_img.at<uint16_t>(v,u) == 0)
      {
        rgb_img.at<cv::Vec3b>(v,u) = color_rgb;           
        depth_img.at<uint16_t>(v,u) = depth; 
      }
      else if  (depth > 0 && depth < depth_img.at<uint16_t>(v,u))
      {
        depth_img.at<uint16_t>(v,u) = depth;
        rgb_img.at<cv::Vec3b>(v,u) = color_rgb;
      }
    }
  }   
}

void tfFromImagePair(
  const cv::Mat& reference_img,
  const cv::Mat& virtual_img,
  const cv::Mat& virtual_depth_img,
  const Matrix3f& intrinsic_matrix,
  tf::Transform& transform)
{

}

} //namespace ccny_rgbd
