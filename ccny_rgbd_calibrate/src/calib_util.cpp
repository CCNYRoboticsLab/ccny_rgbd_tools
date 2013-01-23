#include "ccny_rgbd_calibrate/calib_util.h"

namespace ccny_rgbd {

cv::Mat matrixFromRvecTvec(const cv::Mat& rvec, const cv::Mat& tvec)
{
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  return matrixFromRT(rmat, tvec);
}
 
cv::Mat m4(const cv::Mat& m3)
{
  cv::Mat m4 = cv::Mat::zeros(4, 4, CV_64FC1);

  for (int i = 0; i < 4; ++i)
  for (int j = 0; j < 3; ++j) 
    m4.at<double>(j,i) = m3.at<double>(j,i);

  m4.at<double>(3,3) = 1.0;
  return m4;
}
 
cv::Mat matrixFromRT(const cv::Mat& rmat, const cv::Mat& tvec)
{   
  cv::Mat E = cv::Mat::zeros(3, 4, CV_64FC1);
  
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

  return E;
}
 
void create8bImage(
  const cv::Mat depth_img,
  cv::Mat& depth_img_u)
{
  int w = depth_img.cols;
  int h = depth_img.rows;
  
  depth_img_u = cv::Mat::zeros(h, w, CV_8UC1);
  
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
  const cv::Size& pattern_size, 
  const Point2fVector& corners_2d, 
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
  const Point3fVector& corners_3d_depth,
  const Point2fVector& vertices,
  const cv::Mat& intr_rect_ir,
  cv::Mat& depth_img_g)
{
  int rows = depth_img_g.rows;
  int cols = depth_img_g.cols;
  
  // calculate vertices
  cv::Mat v0(corners_3d_depth[0]);
  cv::Mat v1(corners_3d_depth[1]);
  cv::Mat v2(corners_3d_depth[corners_3d_depth.size()-1]);
  
  v0.convertTo(v0, CV_64FC1);
  v1.convertTo(v1, CV_64FC1);
  v2.convertTo(v2, CV_64FC1);
    
  // calculate normal
  cv::Mat n = (v1 - v0).cross(v2 - v0);
 
  // cache inverted intrinsics
  cv::Mat intr_inv = intr_rect_ir.inv();
  
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
      depth_img_g.at<uint16_t>(v, u) = (int)z;
    } 
  }
}

} // namespace ccny_rgbd
