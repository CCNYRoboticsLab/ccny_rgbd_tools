#include "ccny_rgbd/rgbd_util.h"

namespace ccny_rgbd
{

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
  ret.setOrigin(btVector3(trans(0,3),trans(1,3),trans(2,3)));
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

void getXYZRPY(
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
  //TODO: can be optimized for squared length
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

void holeFilling(const cv::Mat& rgb_img,
                 const cv::Mat& depth_img,
                 uint mask_size,
                 cv::Mat& filled_rgb_img,
                 cv::Mat& filled_depth_img)

{
  uint w = (mask_size-1)/2;

  filled_rgb_img   = cv::Mat::zeros(rgb_img.rows, rgb_img.cols, CV_8UC3);
  filled_depth_img = cv::Mat::zeros(depth_img.rows, depth_img.cols, CV_16UC1);
  
  for (uint u=0; u < depth_img.cols; ++u)
    for (uint v=0; v < depth_img.rows; ++v)
    {
      if (depth_img.at<uint16_t>(v,u) == 0)
      {
        double count = 0;
        double depth_sum = 0;
        double rgb_sum_b = 0; 
        double rgb_sum_g = 0;
        double rgb_sum_r = 0;

        for (uint uu = u - w; uu <= u+w; ++uu)
        for (uint vv = v - w; vv <= v+w; ++vv)
        {
          if (uu < 0 || uu >= depth_img.cols || vv < 0 || vv >= depth_img.rows ) continue;

          uint16_t neighbor_depth  = depth_img.at<uint16_t>(vv, uu);
          cv::Vec3b neighbor_color = rgb_img.at<cv::Vec3b>(vv, uu);

          if (neighbor_depth != 0)
          {
            double neighbor_color_b = (double)neighbor_color[0];
            double neighbor_color_g = (double)neighbor_color[1];
            double neighbor_color_r = (double)neighbor_color[2];

            depth_sum = depth_sum + neighbor_depth;
            rgb_sum_b = rgb_sum_b + neighbor_color_b;
            rgb_sum_g = rgb_sum_g + neighbor_color_g;
            rgb_sum_r = rgb_sum_r + neighbor_color_r;
            ++count;
          }
        }
        if (count != 0)
        { 
          filled_depth_img.at<uint16_t>(v,u) = depth_sum/count;
          cv::Vec3b color_rgb;
          color_rgb[0] = (uint8_t)(rgb_sum_b/count);  
          color_rgb[1] = (uint8_t)(rgb_sum_g/count);
          color_rgb[2] = (uint8_t)(rgb_sum_r/count);
          filled_rgb_img.at<cv::Vec3b>(v,u) = color_rgb;
        }
      }
      else 
      {
        filled_depth_img.at<uint16_t>(v,u) = depth_img.at<uint16_t>(v,u);
        filled_rgb_img.at<cv::Vec3b>(v,u)  = rgb_img.at<cv::Vec3b>(v,u); 
      }
    }
   
}

void holeFilling2(const cv::Mat& rgb_img,
                 const cv::Mat& depth_img,
                 uint mask_size,
                 cv::Mat& filled_rgb_img,
                 cv::Mat& filled_depth_img)

{
  uint w = (mask_size-1)/2;

  filled_rgb_img   = cv::Mat::zeros(rgb_img.rows, rgb_img.cols, CV_8UC3);
  filled_depth_img = cv::Mat::zeros(depth_img.rows, depth_img.cols, CV_16UC1);
  
  for (uint u=0; u < depth_img.cols; ++u)
    for (uint v=0; v < depth_img.rows; ++v)
    {
      if (depth_img.at<uint16_t>(v,u) == 0)
      {
        
        double min_depth = 0;
        for (uint uu = u - w; uu <= u+w; ++uu)
        for (uint vv = v - w; vv <= v+w; ++vv)
        {
          if (uu < 0 || uu >= depth_img.cols || vv < 0 || vv >= depth_img.rows ) continue;
          
          uint16_t neighbor_depth  = depth_img.at<uint16_t>(vv, uu);
          cv::Vec3b neighbor_color = rgb_img.at<cv::Vec3b>(vv, uu);

          if (neighbor_depth != 0 )
          {
            if (min_depth == 0 || neighbor_depth < min_depth )
            {
              min_depth = neighbor_depth;   
              filled_depth_img.at<uint16_t>(v,u) =  min_depth;  
              filled_rgb_img.at<cv::Vec3b>(v,u)  =  rgb_img.at<cv::Vec3b>(vv,uu);       
            }
          }
        }
      }  
      else 
      {
        filled_depth_img.at<uint16_t>(v,u) = depth_img.at<uint16_t>(v,u);
        filled_rgb_img.at<cv::Vec3b>(v,u)  = rgb_img.at<cv::Vec3b>(v,u); 
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
