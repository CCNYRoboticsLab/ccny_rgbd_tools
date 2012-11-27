#include "ccny_rgbd_calibrate/calib_util.h"

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
  
/*
 * Constructs a point cloud, given a depth image which
 * has been undistorted and registered in the rgb frame, 
 * and an rgb image.
 * The intinsic matrix is the RGB matrix after rectification
 * The depth image is uint16_t, in mm
 */
void buildPointCloud(
  const cv::Mat& depth_img_rect_reg,
  const cv::Mat& rgb_img_rect,
  const cv::Mat& intr_rect_rgb,
  PointCloudT& cloud)
{
  int w = depth_img_rect_reg.cols;
  int h = depth_img_rect_reg.rows;
  
  cv::Mat p(3, 1, CV_64FC1);
  PointT pt;
  
  for (int u = 0; u < w; ++u)
  for (int v = 0; v < h; ++v)
  {
    uint16_t  z = depth_img_rect_reg.at<uint16_t>(v, u);
    const cv::Vec3b& c = rgb_img_rect.at<cv::Vec3b>(v, u);
    
    if (z != 0)
    {  
      p.at<double>(0,0) = u;
      p.at<double>(1,0) = v;
      p.at<double>(2,0) = 1.0;
      
      cv::Mat P = z * intr_rect_rgb.inv() * p;   
        
      pt.x = P.at<double>(0);
      pt.y = P.at<double>(1);
      pt.z = P.at<double>(2);
  
      pt.r = c[2];
      pt.g = c[1];
      pt.b = c[0];
    }
    else
    {
      pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
    }

    cloud.points.push_back(pt);
  }  
}

/* reprojects a depth image to another depth image,
 * registered in the rgb camera's frame
 */
void buildRegisteredDepthImage(
  const cv::Mat& intr_rect_ir,
  const cv::Mat& intr_rect_rgb,
  const cv::Mat& rgb2ir,
  const cv::Mat& depth_img_rect,
        cv::Mat& depth_img_rect_reg)
{
  printf("Building regstered image...\n");
  
  int w = depth_img_rect.cols;
  int h = depth_img_rect.rows;
      
  // **** reproject
  
  depth_img_rect_reg = cv::Mat::zeros(h, w, CV_16UC1);
  
  cv::Mat p(3, 1, CV_64FC1);
  
  std::cout << "intr_rect_rgb" << std::endl << intr_rect_rgb << std::endl;
  std::cout << "rgb2ir" << std::endl << rgb2ir << std::endl;
  
  cv::Mat M = intr_rect_rgb * rgb2ir;
  
  for (int u = 0; u < w; ++u)
  for (int v = 0; v < h; ++v)
  {
    double z = (double) depth_img_rect.at<uint16_t>(v,u);

    p.at<double>(0,0) = u;
    p.at<double>(1,0) = v;
    p.at<double>(2,0) = 1.0;
    
    cv::Mat P = z * intr_rect_ir.inv() * p;    
       
    cv::Mat PP(4, 1, CV_64FC1);
    PP.at<double>(0,0) = P.at<double>(0,0);
    PP.at<double>(1,0) = P.at<double>(1,0);
    PP.at<double>(2,0) = P.at<double>(2,0);
    PP.at<double>(3,0) = 1; 
    
    cv::Mat q = M * PP; 
    
    double qx = q.at<double>(0,0);
    double qy = q.at<double>(1,0);
    double qz = q.at<double>(2,0);
    
    int qu = qx / qz;
    int qv = qy / qz;  
    
    // skip outside of image 
    if (qu < 0 || qu >= w || qv < 0 || qv >= h) continue;
    
    uint16_t& val = depth_img_rect_reg.at<uint16_t>(qv, qu);
    
    // z buffering
    if (val == 0 || val > qz) val = qz;
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
  cv::Mat img_corners = img;
  cv::drawChessboardCorners(
  img_corners, pattern_size, cv::Mat(corners_2d), corner_result);
  cv::imshow(title, img_corners);    
}

} // namespace ccny_rgbd
