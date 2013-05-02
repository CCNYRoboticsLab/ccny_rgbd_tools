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

#include "ccny_rgbd/util.h"

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

tf::Transform tfFromEigen(Eigen::Matrix4f t)
{
  tf::Transform tf;
  
  tf::Matrix3x3 m;
  m.setValue(t(0,0),t(0,1),t(0,2),
             t(1,0),t(1,1),t(1,2),
             t(2,0),t(2,1),t(2,2));
  tf.setBasis(m);
  
  tf.setOrigin(tf::Vector3(t(0,3),t(1,3),t(2,3)));
  
  return tf;
}

Eigen::Matrix4f eigenFromTf(const tf::Transform& tf)
{
  Eigen::Matrix4f out_mat;

  double mv[12];
  tf.getBasis().getOpenGLSubMatrix(mv);

  tf::Vector3 origin = tf.getOrigin();

  out_mat(0, 0) = mv[0]; out_mat(0, 1) = mv[4]; out_mat(0, 2) = mv[8];
  out_mat(1, 0) = mv[1]; out_mat(1, 1) = mv[5]; out_mat(1, 2) = mv[9];
  out_mat(2, 0) = mv[2]; out_mat(2, 1) = mv[6]; out_mat(2, 2) = mv[10];

  out_mat(3, 0) = out_mat(3, 1) = out_mat(3, 2) = 0; out_mat(3, 3) = 1;
  out_mat(0, 3) = origin.x();
  out_mat(1, 3) = origin.y();
  out_mat(2, 3) = origin.z();

  return out_mat;
}


tf::Transform tfFromEigenAffine(const AffineTransform& t) 
{ 
  tf::Transform tf;
  
  tf::Matrix3x3 m;
  m.setValue(t(0,0),t(0,1),t(0,2),
             t(1,0),t(1,1),t(1,2),
             t(2,0),t(2,1),t(2,2));
  tf.setBasis(m);
  
  tf.setOrigin(tf::Vector3(t(0,3),t(1,3),t(2,3)));
  
  return tf;
}

AffineTransform eigenAffineFromTf(const tf::Transform& tf)
{
  AffineTransform affine;

  double mv[12];
  tf.getBasis().getOpenGLSubMatrix(mv);

  tf::Vector3 origin = tf.getOrigin();

  affine(0, 0) = mv[0]; affine(0, 1) = mv[4]; affine (0, 2) = mv[8];
  affine(1, 0) = mv[1]; affine(1, 1) = mv[5]; affine (1, 2) = mv[9];
  affine(2, 0) = mv[2]; affine(2, 1) = mv[6]; affine (2, 2) = mv[10];

  affine(3, 0) = affine(3, 1) = affine(3, 2) = 0; affine(3, 3) = 1;
  affine(0, 3) = origin.x();
  affine(1, 3) = origin.y();
  affine(2, 3) = origin.z();

  return affine;
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

void createRGBDFrameFromROSMessages(
  const ImageMsg::ConstPtr& rgb_msg,
  const ImageMsg::ConstPtr& depth_msg,
  const CameraInfoMsg::ConstPtr& info_msg,
  rgbdtools::RGBDFrame& frame)
{
  // prepate opencv rgb image matrix
  cv::Mat rgb_img = cv_bridge::toCvShare(rgb_msg)->image;
  
  // prepate opencv depth image matrix
  // handles 16UC1 natively
  // 32FC1 need to be converted into 16UC1
  cv::Mat depth_img;

  const std::string& enc = depth_msg->encoding; 
  if (enc.compare("16UC1") == 0)
    depth_img = cv_bridge::toCvShare(depth_msg)->image;
  else if (enc.compare("32FC1") == 0)
    rgbdtools::depthImageFloatTo16bit(cv_bridge::toCvShare(depth_msg)->image, depth_img);
    
  // prepare opencv intrinsic matrix from incoming camera info
  cv::Mat intr, dist;
  convertCameraInfoToMats(info_msg, intr, dist);
  /// @todo assert that distortion (dist) is 0
  
  // prepare rgbdtools header from incoming header
  rgbdtools::Header header;
  
  header.seq        = rgb_msg->header.seq;
  header.frame_id   = rgb_msg->header.frame_id;
  header.stamp.sec  = rgb_msg->header.stamp.sec;
  header.stamp.nsec = rgb_msg->header.stamp.nsec;
    
  // initialize the RGBDframe
  frame = rgbdtools::RGBDFrame(rgb_img, depth_img, intr, header);
}

void pathEigenAffineToROS(
  const AffineTransformVector& path,
  PathMsg& path_msg)
{
  assert(path.size() == path_msg.poses.size());

  for (int idx = 0; idx < path.size(); ++idx)
  {
    tf::Transform pose_tf = tfFromEigenAffine(path[idx]);
    tf::poseTFToMsg(pose_tf, path_msg.poses[idx].pose);
  }
}

void pathROSToEigenAffine(
  const PathMsg& path_msg,
  AffineTransformVector& path)
{
  path.clear();
  path.resize(path_msg.poses.size());

  for (int idx = 0; idx < path.size(); ++idx)
  {
    tf::Transform pose_tf;
    tf::poseMsgToTF(path_msg.poses[idx].pose, pose_tf);
    path[idx] = eigenAffineFromTf(pose_tf);
  }
}

} //namespace ccny_rgbd
