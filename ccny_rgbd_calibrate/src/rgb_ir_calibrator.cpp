#include "ccny_rgbd_calibrate/rgb_ir_calibrator.h"

namespace ccny_rgbd {

RGBIRCalibrator::RGBIRCalibrator(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), nh_private_(nh_private)
{  
  std::string home_path = getenv("HOME");
  path_ = home_path + "/ros/ccny-ros-pkg/ccny_rgbd_data/images/ext_calib_01/";
  
  // parameters
  square_size_ = 23.0;
  n_cols_ = 8;
  n_rows_ = 6;
  
  patternsize_ = cv::Size(n_cols_, n_rows_);
  
  // input  
  rgb_test_filename_   = path_ + "test/rgb/0001.png";
  depth_test_filename_ = path_ + "test/depth/0001.png";
  
  calib_rgb_filename_ = path_ + "rgb.yml";
  calib_ir_filename_  = path_ + "depth.yml";
 
  //output
  calib_extrinsic_filename_ = path_ + "extr.yml";
  
  cloud_filename_ = path_ + "cloud.pcd";

  calibrate();
  testExtrinsicCalibration();
 
  // output to screen and file to file
  ROS_INFO("Writing to %s", calib_extrinsic_filename_.c_str()); 
  cv::FileStorage fs(calib_extrinsic_filename_, cv::FileStorage::WRITE);
  fs << "ir2rgb" << ir2rgb_;
}

RGBIRCalibrator::~RGBIRCalibrator()
{
 
}

void RGBIRCalibrator::build3dCornerVector()
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

bool RGBIRCalibrator::loadCalibrationImagePair(
  int idx,
  cv::Mat& rgb_img,
  cv::Mat& ir_img)
{
  // construct the filename image index
  std::stringstream ss_filename;
  ss_filename << std::setw(4) << std::setfill('0') << idx << ".png";

  // construct the rgb and ir file paths
  std::string rgb_filename = path_ + "train/rgb/" + ss_filename.str();
  std::string ir_filename  = path_ + "train/ir/"  + ss_filename.str();

  ROS_INFO("Trying images \n\t%s\n\t%s", rgb_filename.c_str(), ir_filename.c_str());
  
  // check that both exist
  if (!boost::filesystem::exists(rgb_filename))
  {
    return false;
  }
  if (!boost::filesystem::exists(ir_filename))
  {
    ROS_INFO("%s does not exist", ir_filename.c_str());
    return false;
  }
  
  // load the images 
  rgb_img = cv::imread(rgb_filename);
  ir_img  = cv::imread(ir_filename);

  return true;
}

bool RGBIRCalibrator::loadCameraParams()
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

void RGBIRCalibrator::buildRectMaps()
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

void RGBIRCalibrator::calibrate()
{
  if (!loadCameraParams()) return;
  buildRectMaps();
  build3dCornerVector();
  
  std::vector<Point3fVector> v_corners_3d;
  std::vector<Point2fVector> v_corners_2d_rgb;
  std::vector<Point2fVector> v_corners_2d_ir;

  // loop through all test images

  int img_idx = 0;

  while(true)
  {
    // read in calibration images
    cv::Mat rgb_img, ir_img;
    bool load_result = loadCalibrationImagePair(img_idx, rgb_img, ir_img);  
    if (!load_result)
    {
      ROS_INFO("Images not found - reached end of image sequence.");
      break;
    }
    else ROS_INFO("Loaded calibration image pair %d", img_idx);
   
    // rectify
    cv::Mat rgb_img_rect, ir_img_rect; // rectified images 
    cv::remap(rgb_img, rgb_img_rect, map_rgb_1_, map_rgb_2_, cv::INTER_LINEAR);
    cv::remap(ir_img,  ir_img_rect,  map_ir_1_,  map_ir_2_,  cv::INTER_LINEAR);
  
    // detect corners
    ROS_INFO("Detecting 2D corners...");
    Point2fVector corners_2d_rgb, corners_2d_ir;
  
    bool corner_result_rgb = getCorners(rgb_img_rect, patternsize_, corners_2d_rgb);
    bool corner_result_ir  = getCorners(ir_img_rect,  patternsize_, corners_2d_ir);
  
    if (!corner_result_rgb || !corner_result_ir)
    {
      ROS_WARN("Corner detection failed. Skipping image pair %d.", img_idx);
      img_idx++;
      continue;
    }

    // show images with corners
    showCornersImage(rgb_img_rect, patternsize_, corners_2d_rgb, 
                     corner_result_rgb, "RGB Corners");
    showCornersImage(ir_img_rect, patternsize_, corners_2d_ir, 
                     corner_result_ir, "IR Corners");   
    cv::waitKey(500);
   
    // add corners to vectors
    v_corners_3d.push_back(corners_3d_);
    v_corners_2d_rgb.push_back(corners_2d_rgb);
    v_corners_2d_ir.push_back(corners_2d_ir);

    // increment image idnex
    img_idx++;
  }

  // perform the calibration
  ROS_INFO("Determining extrinsic calibration...");

  cv::Mat dist_rect_ir_  = cv::Mat::zeros(1, 5, CV_64FC1);
  cv::Mat dist_rect_rgb_ = cv::Mat::zeros(1, 5, CV_64FC1);

  cv::Mat R, t, E, F;
  double reproj_error = cv::stereoCalibrate(
    v_corners_3d, v_corners_2d_ir, v_corners_2d_rgb,
    intr_rect_ir_,  dist_rect_ir_, 
    intr_rect_rgb_, dist_rect_rgb_,
    cv::Size(), R, t, E, F);
  
  ROS_INFO("Reprojection error: %f", reproj_error);
  
  ir2rgb_ = matrixFromRT(R, t);
}

void RGBIRCalibrator::testExtrinsicCalibration()
{
  // **** load images
  cv::Mat rgb_img    = cv::imread(rgb_test_filename_);
  cv::Mat depth_img  = cv::imread(depth_test_filename_,-1);
 
  // **** rectify
  cv::Mat rgb_img_rect, depth_img_rect;
  cv::remap(rgb_img,   rgb_img_rect,   map_rgb_1_, map_rgb_2_, cv::INTER_LINEAR);
  cv::remap(depth_img, depth_img_rect, map_ir_1_,  map_ir_2_,  cv::INTER_NEAREST);
  
  // **** reproject
  cv::Mat depth_img_rect_reg;
  buildRegisteredDepthImage(intr_rect_ir_, intr_rect_rgb_, ir2rgb_,
                            depth_img_rect, depth_img_rect_reg);
  
  // **** visualize
  showBlendedImage(depth_img_rect,     rgb_img_rect, "Unregistered");
  showBlendedImage(depth_img_rect_reg, rgb_img_rect, "Registered");  
  cv::waitKey(0);
  
  // **** save as a point cloud
  PointCloudT cloud;
  buildPointCloud(depth_img_rect_reg, rgb_img_rect, intr_rect_rgb_, cloud);
  pcl::io::savePCDFileBinary<PointT>(cloud_filename_, cloud);
}

} //namespace ccny_rgbd
