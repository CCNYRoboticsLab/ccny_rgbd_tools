#include "ccny_rgbd_calibrate/depth_calibrator.h"

namespace ccny_rgbd {

DepthCalibrator::DepthCalibrator(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), nh_private_(nh_private)
{  
  std::string home_path = getenv("HOME");
  path_ = home_path + "/ros/ccny-ros-pkg/ccny_rgbd_data/images/ext_calib_01/";
  
  // parameters
  square_size_ = 100.0;
  n_cols_ = 9;
  n_rows_ = 6;
  
  patternsize_ = cv::Size(n_cols_, n_rows_);
  
  // input   
  calib_rgb_filename_ = path_ + "rgb.yml";
  calib_ir_filename_  = path_ + "depth.yml";

  calibrate();
}

DepthCalibrator::~DepthCalibrator()
{
 
}

void DepthCalibrator::build3dCornerVector()
{
  // fill in 3d points
  ROS_INFO("Creating vector of 3D corners...");

  for(int j = 0; j < n_rows_; ++j) // the order matters here
  for(int i = 0; i < n_cols_; ++i)
  {
    float x = i * square_size_;
    float y = j * square_size_; 
    corners_3d_.push_back(cv::Point3f(x, y, 0.0));
  }
}

bool DepthCalibrator::loadCalibrationImagePair(
  int idx,
  cv::Mat& rgb_img,
  cv::Mat& depth_img)
{
  // construct the filename image index
  std::stringstream ss_filename;
  ss_filename << std::setw(4) << std::setfill('0') << idx << ".png";

  // construct the rgb and ir file paths
  std::string rgb_filename   = path_ + "train_depth/rgb/"   + ss_filename.str();
  std::string depth_filename = path_ + "train_depth/depth/" + ss_filename.str();

  ROS_INFO("Trying images \n\t%s\n\t%s", rgb_filename.c_str(), depth_filename.c_str());
  
  // check that both exist
  if (!boost::filesystem::exists(rgb_filename))
  {
    return false;
  }
  if (!boost::filesystem::exists(depth_filename))
  {
    ROS_INFO("%s does not exist", depth_filename.c_str());
    return false;
  }
  
  // load the images 
  rgb_img   = cv::imread(rgb_filename);
  depth_img = cv::imread(depth_filename, -1);

  return true;
}

bool DepthCalibrator::loadCameraParams()
{
  // check that both exist
  if (!boost::filesystem::exists(calib_rgb_filename_))
  {
    ROS_ERROR("Could not open %s", calib_rgb_filename_.c_str());
    return false;
  }
  if (!boost::filesystem::exists(calib_ir_filename_))
  {
    ROS_ERROR("Could not open %s", calib_ir_filename_.c_str());
    return false;
  }

  // **** load intrinsics and distortion coefficients
  ROS_INFO("Reading camera info...");
  cv::FileStorage fs_rgb(calib_rgb_filename_, cv::FileStorage::READ);
  cv::FileStorage fs_ir (calib_ir_filename_,  cv::FileStorage::READ);
  
  fs_rgb["camera_matrix"]           >> intr_rgb_;
  fs_rgb["distortion_coefficients"] >> dist_rgb_;
  fs_ir ["camera_matrix"]           >> intr_ir_;
  fs_ir ["distortion_coefficients"] >> dist_ir_;
  
  return true;
}

void DepthCalibrator::buildRectMaps()
{
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
     
  // determine undistortion maps
  initUndistortRectifyMap(
    intr_rgb_, dist_rgb_, cv::Mat(), intr_rect_rgb_, 
    size_rgb_rect, CV_16SC2, map_rgb_1_, map_rgb_2_);
  
  initUndistortRectifyMap(
    intr_ir_, dist_ir_, cv::Mat(), intr_rect_ir_, 
    size_ir_rect, CV_16SC2, map_ir_1_, map_ir_2_);
}

void DepthCalibrator::calibrate()
{
  if (!loadCameraParams()) return;
  buildRectMaps();
  build3dCornerVector();
  
  // read in calibration images
  int img_idx = 0;
  cv::Mat rgb_img, depth_img;
  bool load_result = loadCalibrationImagePair(img_idx, rgb_img, depth_img);  
  if (!load_result)
  {
    ROS_INFO("Images not found - reached end of image sequence.");
    // break
    return;
  }
  
  // rectify
  cv::Mat rgb_img_rect, depth_img_rect; // rectified images 
  cv::remap(rgb_img,   rgb_img_rect,   map_rgb_1_, map_rgb_2_, cv::INTER_LINEAR);
  cv::remap(depth_img, depth_img_rect, map_ir_1_,  map_ir_2_,  cv::INTER_NEAREST);

  // detect corners
  ROS_INFO("Detecting 2D corners...");
  std::vector<cv::Point2f> corners_2d_rgb;

  bool corner_result_rgb = getCorners(rgb_img_rect, corners_2d_rgb);

  if (!corner_result_rgb)
  {
    ROS_WARN("Corner detection failed. Skipping image pair %d.", img_idx);
    return;
    //img_idx++;
    //continue;
  }
  
  // show images with corners
  if(1)
  {
    cv::Mat rgb_img_rect_corners = rgb_img_rect;
    cv::drawChessboardCorners(
      rgb_img_rect_corners, patternsize_, cv::Mat(corners_2d_rgb), corner_result_rgb);
    cv::imshow("RGB Corners", rgb_img_rect_corners);  
    cv::waitKey(0);
  }
  
  // get pose from RGB camera to checkerboard
  cv::Mat rvec, tvec;
  bool pnp_result = solvePnP(corners_3d_, corners_2d_rgb, 
    intr_rect_rgb_, cv::Mat(), rvec, tvec);

  std::cout << "rvec" << std::endl << rvec << std::endl; 
  std::cout << "tvec" << std::endl << tvec << std::endl;

  testPlaneDetection(rgb_img_rect, depth_img_rect, rvec, tvec);
}

void DepthCalibrator::testPlaneDetection(
  const cv::Mat& rgb_img_rect,
  const cv::Mat& depth_img_rect,
  const cv::Mat& rvec,
  const cv::Mat& tvec)
{
  
  
}

void DepthCalibrator::matrixFromRvecTvec(
  const cv::Mat& rvec,
  const cv::Mat& tvec,
  cv::Mat& E)
{
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  matrixFromRT(rmat, tvec, E);
}

void DepthCalibrator::matrixFromRT(
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

/*
 * Constructs a point cloud, given a depth image which
 * has been undistorted and registered in the rgb frame, 
 * and an rgb image.
 * The intinsic matrix is the RGB matrix after rectification
 * The depth image is uint16_t, in mm
 */

void DepthCalibrator::buildPouintCloud(
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
}

void DepthCalibrator::testExtrinsicCalibration()
{
  // load images
  cv::Mat rgb_img   = cv::imread(rgb_test_filename_);
  cv::Mat depth_img = cv::imread(depth_test_filename_,-1);
  
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

void DepthCalibrator::create8bImage(
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

void DepthCalibrator::blendImages(
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

bool DepthCalibrator::getCorners(
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
