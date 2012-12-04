#include "ccny_rgbd_calibrate/calib_util.h"

#include <ros/ros.h>

namespace ccny_rgbd {
  
void create8bImage(
  const cv::Mat depth_img,
  cv::Mat& depth_img_u)
{
  int w = depth_img.cols;
  int h = depth_img.rows;
  
  depth_img_u = cv::Mat::zeros(h,w, CV_8UC1);
  
  for (int u = 0; u < w; ++u)
  for (int v = 0; v < h; ++v)
  {
    uint16_t d = depth_img.at<uint16_t>(v,u);
    double df =((double)d * 0.001) * 100.0;
    depth_img_u.at<uint8_t>(v,u) = (int)df;
  }
}

void blendImages(
  const cv::Mat& rgb_img,
  const cv::Mat depth_img,
  cv::Mat& blend_img)
{
  double scale = 0.2;
  
  int w = rgb_img.cols;
  int h = rgb_img.rows;
  
  blend_img = cv::Mat::zeros(h,w, CV_8UC3);
  
  for (int u = 0; u < w; ++u)
  for (int v = 0; v < h; ++v)
  {
    cv::Vec3b color_rgb = rgb_img.at<cv::Vec3b>(v,u);
    uint8_t mono_d = depth_img.at<uint8_t>(v, u);   
    cv::Vec3b color_d(mono_d, mono_d, mono_d);
    blend_img.at<cv::Vec3b>(v,u) = scale*color_rgb + (1.0-scale)*color_d;
  }
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
  const cv::Mat& rgb2ir,
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
  Eigen::Matrix<double, 3, 4> rgb2ir_eigen;
  for (int u = 0; u < 4; ++u)
  for (int v = 0; v < 3; ++v)
    rgb2ir_eigen(v,u) =  rgb2ir.at<double>(v, u);
   
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
    intr_rect_rgb_eigen * (rgb2ir_eigen * intr_rect_ir_inv_eigen);

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

void matrixFromRvecTvec(
  const cv::Mat& rvec,
  const cv::Mat& tvec,
  cv::Mat& E)
{
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  matrixFromRT(rmat, tvec, E);
}

void matrixFromRT(
  const cv::Mat& rmat,
  const cv::Mat& tvec,
  cv::Mat& E)
{   
  E = cv::Mat::zeros(3, 4, CV_64FC1);
  
  E.at<double>(0,0) = rmat.at<double>(0,0);
  E.at<double>(0,1) = rmat.at<double>(0,1);
  E.at<double>(0,2) = rmat.at<double>(0,2);
  E.at<double>(1,0) = rmat.at<double>(1,0);
  E.at<double>(1,1) = rmat.at<double>(1,1);
  E.at<double>(1,2) = rmat.at<double>(1,2);
  E.at<double>(2,0) = rmat.at<double>(2,0);
  E.at<double>(2,1) = rmat.at<double>(2,1);
  E.at<double>(2,2) = rmat.at<double>(2,2);

  E.at<double>(0,3) = tvec.at<double>(0,0);
  E.at<double>(1,3) = tvec.at<double>(1,0);
  E.at<double>(2,3) = tvec.at<double>(2,0);
}

bool getCorners(
  const cv::Mat& img,
  const cv::Size& pattern_size,
  Point2fVector& corners)
{
  // convert to mono
  cv::Mat img_mono;
  cv::cvtColor(img, img_mono, CV_RGB2GRAY);
 
  int params = CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FAST_CHECK;
  
  bool found = findChessboardCorners(img_mono, pattern_size, corners, params);
  
  if(found)
    cornerSubPix(
      img_mono, corners, pattern_size, cv::Size(-1, -1),
      cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
  
  return found;
}
  
void showBlendedImage(
  const cv::Mat& depth_img,
  const cv::Mat& rgb_img,
  const std::string& title)
{
  // create 8b images
  cv::Mat depth_img_u;
  create8bImage(depth_img, depth_img_u);
  
  // blend and show
  cv::Mat blend_img;
  blendImages(rgb_img, depth_img_u, blend_img); 
  cv::imshow(title, blend_img);
}

void showCornersImage(
  const cv::Mat& img, 
  const cv::Size pattern_size, 
  const Point2fVector corners_2d, 
  bool corner_result,
  const std::string title)
{ 
  cv::Mat img_corners = img.clone();
  cv::drawChessboardCorners(
  img_corners, pattern_size, cv::Mat(corners_2d), corner_result);
  cv::imshow(title, img_corners);    
}

void getCheckerBoardPolygon(
  const Point2fVector& corners_2d,
  int n_rows, int n_cols,
  Point2fVector& vertices)
{
  cv::Point2f d = (corners_2d[0] - corners_2d[n_cols+1])*0.6; 
  vertices.resize(4);
  
  vertices[0] = cv::Point2f(+d.x, +d.y) + corners_2d[0];
  vertices[1] = cv::Point2f(-d.y, +d.x) + corners_2d[n_cols - 1];
  vertices[2] = cv::Point2f(-d.x, -d.y) + corners_2d[n_cols * n_rows - 1];
  vertices[3] = cv::Point2f(+d.y, -d.x) + corners_2d[n_cols * (n_rows - 1)];
}

void buildCheckerboardDepthImage(
  const Point3fVector& corners_3d,
  const Point2fVector& vertices,
  int rows, int cols,
  const cv::Mat& rvec, const cv::Mat& tvec,
  const cv::Mat& intr_rect_rgb,
  cv::Mat& depth_img)
{
  depth_img = cv::Mat::zeros(rows, cols, CV_16UC1);
  
  // calculate vertices
  cv::Mat v0(corners_3d[0]);
  cv::Mat v1(corners_3d[1]);
  cv::Mat v2(corners_3d[corners_3d.size()-1]);
  
  v0.convertTo(v0, CV_64FC1);
  v1.convertTo(v1, CV_64FC1);
  v2.convertTo(v2, CV_64FC1);
  
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  
  // rotate into camera frame
  v0 = rmat * v0 + tvec;
  v1 = rmat * v1 + tvec;
  v2 = rmat * v2 + tvec;
  
  // calculate normal
  cv::Mat n = (v1 - v0).cross(v2 - v0);
 
  // cache inverted intrinsics
  cv::Mat intr_inv = intr_rect_rgb.inv();
  
  cv::Mat q = cv::Mat::zeros(3, 1, CV_64FC1);
  PointT pt;
  
  for (int u = 0; u < cols; ++u)  
  for (int v = 0; v < rows; ++v)
  {    
    float dist = cv::pointPolygonTest(vertices, cv::Point2f(u,v), false);
    if (dist > 0)
    {
      q.at<double>(0,0) = (double)u;
      q.at<double>(1,0) = (double)v;
      q.at<double>(2,0) = 1.0;
      
      q = intr_inv * q;
      double d = v0.dot(n) / q.dot(n);
      cv::Mat p = d * q;

      double z = p.at<double>(2,0);
      depth_img.at<uint16_t>(v, u) = (int)z;
    } 
  }
}

void unwarpDepthImage(
  cv::Mat& depth_img,
  const cv::Mat& coeff0,
  const cv::Mat& coeff1,
  const cv::Mat& coeff2)
{
  /*
  // TODO: This is faster (5ms vs 4 ms)
  // BUT images need to be passed as CV_64FC1 or CV_32FC1
  // currently the conversion to float messes up 0 <-> nan values
  
  cv::Mat temp;
  depth_img.convertTo(temp, CV_64FC1);
  temp = coeff0 + temp.mul(coeff1 + temp.mul(coeff2));   
  temp.convertTo(depth_img, CV_16UC1);
  
  return;
  */

  for (int u = 0; u < depth_img.cols; ++u)
  for (int v = 0; v < depth_img.rows; ++v)    
  {
    //printf("%d %d\n", u, v);
    uint16_t d = depth_img.at<uint16_t>(v, u);
    
    if (d != 0)
    {
      double c0 = coeff0.at<double>(v, u);
      double c1 = coeff1.at<double>(v, u);
      double c2 = coeff2.at<double>(v, u);
    
      uint16_t res = (int)(c0 + d*(c1 + d*c2));
      depth_img.at<uint16_t>(v, u) = res;
    }
  }
}



} // namespace ccny_rgbd
