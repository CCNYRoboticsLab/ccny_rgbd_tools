#include "ccny_rgbd_calibrate/rgb_ir_calibrator.h"

namespace ccny_rgbd {

RGBIRCalibrator::RGBIRCalibrator(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), nh_private_(nh_private)
{  
  std::string path = "/home/idyanov/ros/images/ext_calib_01/";
  
  // parameters
  square_size_ = 23.0;
  n_cols_ = 8;
  n_rows_ = 6;
  
  patternsize_ = cv::Size(n_cols_, n_rows_);
  
  // input
  rgb_filename_ = path + "rgb.png";
  ir_filename_  = path + "ir.png"; 
  
  rgb_test_filename_   = path + "test/rgb_01.png";
  depth_test_filename_ = path + "test/depth_01.png";
  
  calib_rgb_filename_ = path + "rgb.yml";
  calib_ir_filename_  = path + "depth.yml";
 
  //output
  calib_extrinsic_filename_ = path + "extr.yml";
  
  cloud_filename_ = path + "cloud.pcd";

  build3dCornerVector();
  calibrate();
   
  // output to screen and file to file
  printf("Writing to %s\n", calib_extrinsic_filename_.c_str()); 
  cv::FileStorage fs(calib_extrinsic_filename_, cv::FileStorage::WRITE);
  fs << "rgb2ir" << rgb2ir_;
  
  testExtrinsicCalibration();
}

RGBIRCalibrator::~RGBIRCalibrator()
{
 
}

void RGBIRCalibrator::build3dCornerVector()
{
  // fill in 3d points
  printf("Creating 3D corners...\n");

  for(int j = 0; j < n_rows_; ++j) // the order matters here
  for(int i = 0; i < n_cols_; ++i)
  {
    float x = i * square_size_;
    float y = j * square_size_; 
    corners_3d_.push_back(cv::Point3f(x, y, 0.0));
  }
}

void RGBIRCalibrator::calibrate()
{
  // **** calibration images
  printf("Reading clibration image...\n");
  cv::Mat rgb_img = cv::imread(rgb_filename_);
  cv::Mat ir_img  = cv::imread(ir_filename_);
   
  //cv::imshow("rgb_img", rgb_img);
  //cv::imshow("ir_img",  ir_img);
  //cv::waitKey(0);
  
  // **** load intrinsics and distortion coefficients
  printf("Reading camera info...\n");
  cv::FileStorage fs_rgb(calib_rgb_filename_, cv::FileStorage::READ);
  cv::FileStorage fs_ir(calib_ir_filename_, cv::FileStorage::READ);
  
  fs_rgb["camera_matrix"]           >> intr_rgb_;
  fs_rgb["distortion_coefficients"] >> dist_rgb_;
  fs_ir["camera_matrix"]           >> intr_ir_;
  fs_ir["distortion_coefficients"] >> dist_ir_;
  
  std::cout << "------- Initrinsic RGB --------" << std::endl;
  std::cout << intr_rgb_ << std::endl;
  
  std::cout << "------- Initrinsic IR --------" << std::endl;
  std::cout << intr_ir_ << std::endl;
  
  // **** build rectification maps

  // determine new matrix
  
  cv::Size size_rgb(640, 480);
  cv::Size size_ir (640, 480);
  double alpha = 0.0;
  
  cv::Size size_rgb_rect(640, 480);
  cv::Size size_ir_rect (640, 480);
  
  intr_rect_rgb_ = getOptimalNewCameraMatrix(
    intr_rgb_, dist_rgb_, size_rgb, alpha, size_rgb_rect);

  intr_rect_ir_ = getOptimalNewCameraMatrix(
    intr_ir_, dist_ir_, size_ir, alpha, size_ir_rect);
     
  // determine undistortion map
 
  initUndistortRectifyMap(
    intr_rgb_, dist_rgb_, cv::Mat(), intr_rect_rgb_, 
    size_rgb_rect, CV_16SC2, map_rgb_1_, map_rgb_2_);
  
  initUndistortRectifyMap(
    intr_ir_, dist_ir_, cv::Mat(), intr_rect_ir_, 
    size_ir_rect, CV_16SC2, map_ir_1_, map_ir_2_);
  
  // rectify
  cv::Mat rgb_img_rect, ir_img_rect; // rectified images
  
  cv::remap(rgb_img, rgb_img_rect, map_rgb_1_, map_rgb_2_, cv::INTER_LINEAR);
  cv::remap(ir_img,  ir_img_rect,  map_ir_1_,  map_ir_2_,  cv::INTER_LINEAR);
  
  // show undistorted images
  //cv::imshow("rgb_img_rect", rgb_img_rect);
  //cv::imshow("ir_img_rect",  ir_img_rect);
  //cv::waitKey(0); 
  
  // **** detect corners
  printf("Detecting 2D corners...\n");
  std::vector<cv::Point2f> corners_2d_rgb, corners_2d_ir;
  
  bool result_rgb = getCorners(rgb_img_rect, corners_2d_rgb);
  bool result_ir  = getCorners(ir_img_rect,  corners_2d_ir);
  
  cv::Mat rgb_img_rect_corners = rgb_img_rect;
  cv::Mat ir_img_rect_corners  = ir_img_rect;
  
  // show images with corners
  cv::drawChessboardCorners(
    rgb_img_rect_corners, patternsize_, cv::Mat(corners_2d_rgb), result_rgb);
  cv::drawChessboardCorners(
    ir_img_rect_corners, patternsize_, cv::Mat(corners_2d_ir), result_ir);
  cv::imshow("RGB Corners", rgb_img_rect_corners);  
  cv::imshow("IR Corners",  ir_img_rect_corners); 
  cv::waitKey(0);
  
  // **** Solve Pnp
  /*
  printf("Solving PnP (RGB)...\n"); 
  cv::Mat rvec_rgb, tvec_rgb;
  cv::Mat rvec_ir, tvec_ir;
 
  solvePnP(
    corners_3d_, corners_2d_rgb, intr_rgb_, dist_rgb_, 
    rvec_rgb, tvec_rgb);
  
  solvePnP(
    corners_3d_, corners_2d_ir, intr_ir_, dist_ir_, 
    rvec_ir, tvec_ir);
  
  matrixFromRvecTvec(rvec_rgb, tvec_rgb, extr_rgb_);
  matrixFromRvecTvec(rvec_ir,  tvec_ir,  extr_ir_);
  
  cv::Mat R_rgb;
  cv::Rodrigues(rvec_rgb, R_rgb);
  
  std::cout << "------- Extrinsic RGB --------" << std::endl;
  std::cout << extr_rgb_ << std::endl;
  
  std::cout << "------- Extrinsic IR ---------" << std::endl;
  std::cout << extr_ir_ << std::endl;

 
 // **** visualize PnP results by reprojecting points
  
  std::vector<cv::Point2f> corners_proj_rgb;
  std::vector<cv::Point2f> corners_proj_ir;
 
  cv::projectPoints(
    corners_3d_, rvec_rgb, tvec_rgb, intr_rgb_, dist_rgb_, corners_proj_rgb);
  cv::projectPoints(
    corners_3d_, rvec_ir,  tvec_ir,  intr_ir_,  dist_ir_,  corners_proj_ir);
  
  cv::Mat proj_rgb_img = rgb_img;
  cv::drawChessboardCorners(
    proj_rgb_img, cv::Size(n_cols_, n_rows_), corners_proj_rgb, true);
  cv::imshow("proj_rgb_img", proj_rgb_img);

  cv::Mat proj_ir_img = ir_img;
  cv::drawChessboardCorners(
    proj_ir_img, cv::Size(n_cols_, n_rows_), corners_proj_ir, true);
  cv::imshow("proj_ir_img", proj_ir_img);
  
  cv::waitKey(0);
    */
  
  // **** stereo calibrate
  printf("Stereo calibrate...\n"); 
  
  typedef std::vector<cv::Point2f> Point2fVector;
  typedef std::vector<cv::Point3f> Point3fVector;
  
  std::vector<Point3fVector> v_corners_3d;
  std::vector<Point2fVector> v_corners_2d_rgb;
  std::vector<Point2fVector> v_corners_2d_ir;
  
  v_corners_3d.push_back(corners_3d_);
  v_corners_3d.push_back(corners_3d_);
  
  v_corners_2d_rgb.push_back(corners_2d_rgb);
  v_corners_2d_rgb.push_back(corners_2d_rgb);
  
  v_corners_2d_ir.push_back(corners_2d_ir);
  v_corners_2d_ir.push_back(corners_2d_ir);
  
  cv::Mat dist_rect_ir_  = cv::Mat::zeros(1, 5, CV_64FC1);
  cv::Mat dist_rect_rgb_ = cv::Mat::zeros(1, 5, CV_64FC1);
  
  cv::Mat R, t, E, F;
  cv::stereoCalibrate(
    v_corners_3d, v_corners_2d_ir, v_corners_2d_rgb,
    intr_rect_ir_,  dist_rect_ir_, 
    intr_rect_rgb_, dist_rect_rgb_,
    cv::Size(), R, t, E, F);
  
  //std::cout << "------- R ---------" << std::endl;
  //std::cout << R_ << std::endl; 
  //std::cout << "------- T ---------" << std::endl;
  //std::cout << T_ << std::endl; 
  //std::cout << "------- E ---------" << std::endl;
  //std::cout << E << std::endl; 
  //std::cout << "------- F ---------" << std::endl;
  //std::cout << F << std::endl;

  matrixFromRT(R, t, rgb2ir_);
}

void RGBIRCalibrator::matrixFromRvecTvec(
  const cv::Mat& rvec,
  const cv::Mat& tvec,
  cv::Mat& E)
{
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  matrixFromRT(rmat, tvec, E);
}

void RGBIRCalibrator::matrixFromRT(
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

void RGBIRCalibrator::testExtrinsicCalibration()
{
  // load images
  cv::Mat rgb_img    = cv::imread(rgb_test_filename_);
  cv::Mat depth_img  = cv::imread(depth_test_filename_,-1);
  
  int w = rgb_img.cols;
  int h = rgb_img.rows; 
  
  // **** rectify

  cv::Mat rgb_img_rect, depth_img_rect;
  
  cv::remap(rgb_img,   rgb_img_rect,   map_rgb_1_, map_rgb_2_, cv::INTER_LINEAR);
  cv::remap(depth_img, depth_img_rect, map_ir_1_,  map_ir_2_,  cv::INTER_NEAREST);
  
  // **** reproject
  
  cv::Mat depth_img_rect_reg = cv::Mat::zeros(h, w, CV_16UC1);
  
  cv::Mat p(3, 1, CV_64FC1);
  
  cv::Mat M = intr_rect_rgb_ * rgb2ir_;
  
  for (int u = 0; u < w; ++u)
  for (int v = 0; v < h; ++v)
  {
    double z = (double) depth_img_rect.at<uint16_t>(v,u);

    p.at<double>(0,0) = u;
    p.at<double>(1,0) = v;
    p.at<double>(2,0) = 1.0;
    
    cv::Mat P = z * intr_rect_ir_.inv() * p;    
    
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
  
  // **** visualize
  
  // create 8b images
  cv::Mat depth_img_rect_u, depth_img_rect_reg_u;
  create8bImage(depth_img_rect,     depth_img_rect_u);
  create8bImage(depth_img_rect_reg, depth_img_rect_reg_u);
  
  // blend and show
  cv::Mat blend_img, blend_img_reg;
  blendImages(rgb_img_rect, depth_img_rect_u,     blend_img);
  blendImages(rgb_img_rect, depth_img_rect_reg_u, blend_img_reg);
  
  cv::imshow("blend_img",     blend_img);
  cv::imshow("blend_img_reg", blend_img_reg);
  cv::waitKey(0);
  
  // **** save as a point cloud
  
  PointCloudT cloud;
  
  for (int u = 0; u < w; ++u)
  for (int v = 0; v < h; ++v)
  {
    uint16_t& z  = depth_img_rect_reg.at<uint16_t>(v, u);
    cv::Vec3b& c = rgb_img_rect.at<cv::Vec3b>(v, u);
    
    PointT pt;
    
    if (z != 0)
    {  
      p.at<double>(0,0) = u;
      p.at<double>(1,0) = v;
      p.at<double>(2,0) = 1.0;
      
      cv::Mat P = z * intr_rect_rgb_.inv() * p;   
        
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
  
  pcl::io::savePCDFileBinary<PointT>(cloud_filename_, cloud);
}

void RGBIRCalibrator::create8bImage(
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

void RGBIRCalibrator::blendImages(
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
  
  cv::imshow("blend_img", blend_img);  
}

bool RGBIRCalibrator::getCorners(
  const cv::Mat& img,
  std::vector<cv::Point2f>& corners)
{
  // convert to mono
  cv::Mat img_mono;
  cv::cvtColor(img, img_mono, CV_RGB2GRAY);
 
  int params = CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE + CV_CALIB_CB_FAST_CHECK;
  
  bool found = findChessboardCorners(img_mono, patternsize_, corners, params);
  
  if(found)
    cornerSubPix(
      img_mono, corners, patternsize_, cv::Size(-1, -1),
      cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
  
  return found;
}

} //namespace ccny_rgbd
