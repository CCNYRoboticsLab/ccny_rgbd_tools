#include "ccny_rgbd_calibrate/depth_calibrator.h"

namespace ccny_rgbd {

DepthCalibrator::DepthCalibrator(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), nh_private_(nh_private)
{  
  // parameters
  if (!nh_private_.getParam ("fit_window_size", fit_window_size_))
    fit_window_size_ = 10;
  if (!nh_private_.getParam ("fit_mode", fit_mode_))
    fit_mode_ = DEPTH_FIT_QUADRATIC_ZERO;
  if (!nh_private_.getParam ("n_cols", n_cols_))
    ROS_ERROR("n_cols param needs to be set");
  if (!nh_private_.getParam ("n_rows", n_rows_))
    ROS_ERROR("n_rows param needs to be set");
  if (!nh_private_.getParam ("square_size", square_size_))
    ROS_ERROR("square_size needs to be set");
  if (!nh_private_.getParam ("path", path_))
    ROS_ERROR("path param needs to be set");

  patternsize_ = cv::Size(n_cols_, n_rows_);
 
  // directories
  cloud_path_ = path_ + "/warp_train/clouds";   
  train_path_ = path_ + "/warp_train";
  test_path_  = path_ + "/warp_test";

  // prepare directories
  boost::filesystem::create_directory(cloud_path_); 

  // calibration filenames   
  calib_rgb_filename_  = path_ + "/rgb.yml";
  calib_ir_filename_   = path_ + "/depth.yml";
  calib_extr_filename_ = path_ + "/extr.yml";
  calib_warp_filename_ = path_ + "/warp.yml";

  depth_test_filename_ = test_path_ + "/depth/0052.png";
  rgb_test_filename_   = test_path_ + "/rgb/0052.png";

  // perform calibration and test it
  calibrate();
  testDepthCalibration();
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
  std::string rgb_filename   = train_path_ + "/rgb/"   + ss_filename.str();
  std::string depth_filename = train_path_ + "/depth/" + ss_filename.str();

  // check that both exist
  if (!boost::filesystem::exists(rgb_filename))
  {
    ROS_INFO("%s does not exist", rgb_filename.c_str());
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
  if (!boost::filesystem::exists(calib_extr_filename_))
  {
    ROS_ERROR("Could not open %s", calib_extr_filename_.c_str());
    return false;
  }

  // load intrinsics and distortion coefficients
  ROS_INFO("Reading camera info...");
  cv::FileStorage fs_rgb (calib_rgb_filename_,  cv::FileStorage::READ);
  cv::FileStorage fs_ir  (calib_ir_filename_,   cv::FileStorage::READ);
  cv::FileStorage fs_extr(calib_extr_filename_, cv::FileStorage::READ);
    
  fs_rgb["camera_matrix"]           >> intr_rgb_;
  fs_rgb["distortion_coefficients"] >> dist_rgb_;
  fs_ir ["camera_matrix"]           >> intr_ir_;
  fs_ir ["distortion_coefficients"] >> dist_ir_;
  
  fs_extr["ir2rgb"] >> ir2rgb_;
  
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
  
  cv::Mat count_img = cv::Mat::zeros(480, 640, CV_8UC1);
  
  std::vector<ReadingVector> readings;
  readings.resize(640*480);
  
  std::vector<DoubleVector> coeffs;
  coeffs.resize(640*480);
  
  cv::Mat c0 = cv::Mat::zeros(480, 640, CV_64FC1);
  cv::Mat c1 = cv::Mat::zeros(480, 640, CV_64FC1);
  cv::Mat c2 = cv::Mat::zeros(480, 640, CV_64FC1);
  
  int img_idx = 0;
  while(true)
  {
    cv::Mat rgb_img, depth_img;
    bool load_result = loadCalibrationImagePair(img_idx, rgb_img, depth_img);  
    if (!load_result)
    {
      ROS_INFO("Images %d not found. Assuming end of sequence", img_idx);
      break;
    }
    
    // process the images
    cv::Mat depth_img_g, depth_img_m;
    bool result = processTrainingImagePair(
      img_idx, rgb_img, depth_img, depth_img_g, depth_img_m);
    if (result)
    {  
      // accumulate 
      for (int u = 0; u < 640; ++u)
      for (int v = 0; v < 480; ++v)
      {
        int idx = v * 640 + u; 
        uint16_t mz = depth_img_m.at<uint16_t>(v, u);
        uint16_t gz = depth_img_g.at<uint16_t>(v, u);      
        
        if (mz != 0 && gz != 0)
        {
          count_img.at<uint8_t>(v, u) += 10;
        
          ReadingVector& vec = readings[idx];
        
          ReadingPair p;
          p.ground_truth = gz;
          p.measured = mz;
          vec.push_back(p);
        }
      }
    }
    
    cv::imshow("Observation density", count_img);
    cv::waitKey(1);
       
    // increment image index
    img_idx++;
  }
  
  // perform fitting
  for(int v = 0; v < 480; ++v) 
  {
    ROS_INFO("Fitting image row [ %.3d ]", v);

    for(int u = 0; u < 640; ++u)
    {
      int idx = v * 640 + u; 
       
      // acumulate data points from neighbors
      ReadingVector vec;
      for(int uu = u - fit_window_size_; uu <= u + fit_window_size_; ++uu)
      for(int vv = v - fit_window_size_; vv <= v + fit_window_size_; ++vv)
      {    
        if (uu < 0 || uu >=640 || vv < 0 || vv >= 480) continue;
             
        int idx_n = vv * 640 + uu; 
        
        const ReadingVector& vec_n = readings[idx_n];
        vec.insert(vec.end(), vec_n.begin(), vec_n.end());
      }

      DoubleVector& coeff = coeffs[idx];       

      fitData(vec, coeff);

      c0.at<double>(v, u) = coeff[0];
      c1.at<double>(v, u) = coeff[1];
      c2.at<double>(v, u) = coeff[2];
    }
  }
  
  // **** write to file filename
  ROS_INFO("Writing to file...");
  std::ofstream datafile;
  std::string data_filename = train_path_ + "/data.txt";
  datafile.open(data_filename.c_str());
  
  for (int idx = 0; idx < 640*480; ++idx)
  {
    int u = idx % 640;
    int v = idx / 640;

    const ReadingVector& vec = readings[idx];
    const DoubleVector& coeff = coeffs[idx]; 
    
    datafile << u << " " << v << " ";
    datafile << coeff[0] << " " << coeff[1] << " " << coeff[2];
    
    for (unsigned int j = 0; j < vec.size(); ++j)
    {
      datafile << " " << vec[j].ground_truth;
      datafile << " " << vec[j].measured;
    }
    
    datafile << std::endl;
  }
  
  datafile.close();
  
  // **** write to yaml file
  ROS_INFO("Writing to %s", calib_warp_filename_.c_str()); 
  cv::FileStorage fs(calib_warp_filename_, cv::FileStorage::WRITE);
  fs << "fit_mode" << fit_mode_;
  fs << "c0" << c0;
  fs << "c1" << c1;
  fs << "c2" << c2;
}

bool DepthCalibrator::processTrainingImagePair(
  int img_idx,
  const cv::Mat& rgb_img,
  const cv::Mat& depth_img,
  cv::Mat& depth_img_g,
  cv::Mat& depth_img_m)
{
  ROS_INFO("Processing image pair [ %.4d ]", img_idx);
  
  depth_img_g = cv::Mat::zeros(depth_img.size(), CV_16UC1);
  depth_img_m = cv::Mat::zeros(depth_img.size(), CV_16UC1);
  
  // **** rectify both images
  cv::Mat rgb_img_rect, depth_img_rect;
  cv::remap(rgb_img,   rgb_img_rect,   map_rgb_1_, map_rgb_2_, cv::INTER_LINEAR);
  cv::remap(depth_img, depth_img_rect, map_ir_1_,  map_ir_2_,  cv::INTER_NEAREST);

  // **** detect corners in RGB image
  std::vector<cv::Point2f> corners_2d_rgb;
  bool corner_result_rgb = getCorners(rgb_img_rect, patternsize_, corners_2d_rgb);
  if (!corner_result_rgb)
  {
    ROS_WARN("Corner detection failed. Skipping image pair");
    return false;
  }
  
  // show images with corners
  showCornersImage(rgb_img_rect, patternsize_, corners_2d_rgb, corner_result_rgb, "RGB Corners");
  cv::waitKey(1);
  
  // **** recover get pose from Depth camera to checkerboard such that:
  // P_IR  = board2ir  * P_Board
  // P_RGB = board2rgb * P_Board
  // board2ir = ir2rgb^-1 * board2rgb
  cv::Mat rvec, tvec;
  bool pnp_result = solvePnP(corners_3d_, corners_2d_rgb, 
    intr_rect_rgb_, cv::Mat(), rvec, tvec);

  if (!pnp_result)
  {
    ROS_WARN("PnP failed. Skipping image pair.");
    return false;
  }
  
  cv::Mat board2rgb = matrixFromRvecTvec(rvec, tvec);  
  cv::Mat board2ir = m4(ir2rgb_).inv() * m4(board2rgb);

  // **** express corners in depth frame
  Point3fVector corners_3d_depth;
  for (unsigned int idx = 0; idx < corners_3d_.size(); ++idx)
  {
    const cv::Point3f& corner_board = corners_3d_[idx];
    cv::Mat P_board(4, 1, CV_64FC1);
    P_board.at<double>(0,0) = corner_board.x;
    P_board.at<double>(1,0) = corner_board.y;
    P_board.at<double>(2,0) = corner_board.z;
    P_board.at<double>(3,0) = 1.0;

    cv::Mat P_depth = board2ir * P_board;

    cv::Point3f corner_depth;
    corner_depth.x = P_depth.at<double>(0,0);
    corner_depth.y = P_depth.at<double>(1,0);
    corner_depth.z = P_depth.at<double>(2,0);

    corners_3d_depth.push_back(corner_depth);
  }

  // **** project corners onto depth image
  Point2fVector corners_2d_depth;
  for (unsigned int idx = 0; idx < corners_3d_depth.size(); ++idx)
  {
    const cv::Point3f& corner_3d = corners_3d_depth[idx];
    cv::Mat P_depth(corner_3d);
    P_depth.convertTo(P_depth, CV_64FC1);

    cv::Mat q_depth = intr_rect_ir_ * P_depth;

    cv::Point2f corner_2d;
    double z = q_depth.at<double>(2,0);

    corner_2d.x = q_depth.at<double>(0,0) / z;
    corner_2d.y = q_depth.at<double>(1,0) / z;

    corners_2d_depth.push_back(corner_2d);
  }

  //showCornersImage(depth_img_rect, patternsize_, corners_2d_depth, true, "reprojected corners");
  //cv::waitKey(1);

  // **** filter depth image by contours
  Point2fVector vertices;
  getCheckerBoardPolygon(corners_2d_depth, n_rows_, n_cols_, vertices);
  
  // depth_img_m is built from depth_img_rect using the polygon mask
  for (int u = 0; u < depth_img_rect.cols; ++u)  
  for (int v = 0; v < depth_img_rect.rows; ++v)
  {
    float dist = cv::pointPolygonTest(vertices, cv::Point2f(u,v), true);
    if (dist <= 0)
      depth_img_m.at<uint16_t>(v, u) = 0;
    else
      depth_img_m.at<uint16_t>(v, u) = depth_img_rect.at<uint16_t>(v, u);
  }
 
  // **** build ground-truth depth image from checkerboard
  buildCheckerboardDepthImage(
    corners_3d_depth, vertices, intr_rect_ir_, depth_img_g);

  // **** build ground-truth and measured point clouds
  PointCloudT g_cloud, m_cloud;
  buildPointCloud(depth_img_g, intr_rect_ir_, g_cloud);
  buildPointCloud(depth_img_m, intr_rect_ir_, m_cloud);
  
  // **** save clouds
  std::stringstream ss_filename;
  ss_filename << std::setw(4) << std::setfill('0') << img_idx << ".pcd";
  
  std::string m_cloud_filename = cloud_path_ + "/m_" + ss_filename.str();
  std::string g_cloud_filename = cloud_path_ + "/g_" + ss_filename.str();
  
  pcl::io::savePCDFileBinary<PointT>(m_cloud_filename, m_cloud);
  pcl::io::savePCDFileBinary<PointT>(g_cloud_filename, g_cloud); 
  
  return true;
}


void DepthCalibrator::fitData(
  const ReadingVector& v, 
  std::vector<double>& coeff)
{
  coeff.resize(3);

  // no enough data
  if (v.size() < 3)
  {
    coeff[0] = 0.0;
    coeff[1] = 1.0;
    coeff[2] = 0.0;
    return;
  }

  if (fit_mode_ == DEPTH_FIT_LINEAR)
    linearFit(v, coeff);
  if (fit_mode_ == DEPTH_FIT_LINEAR_ZERO)
    linearFitZero(v, coeff);
  if (fit_mode_ == DEPTH_FIT_QUADRATIC)
    quadraticFit(v, coeff);
  if (fit_mode_ == DEPTH_FIT_QUADRATIC_ZERO)
    quadraticFitZero(v, coeff);
}

void DepthCalibrator::linearFitZero(
  const ReadingVector& v,
  std::vector<double>& coeff)
{
  int n = v.size();
  double x[n];
  double y[n];
  double c1, cov11, sumsq;

  for(int i = 0; i < n; ++i)
  {  
    x[i] = v[i].measured;
    y[i] = v[i].ground_truth;
  }

  gsl_fit_mul(x, 1, y, 1, n, &c1, &cov11, &sumsq);

  coeff[0] = 0.0;
  coeff[1] = c1;
  coeff[2] = 0.0;
}

void DepthCalibrator::quadraticFitZero(
  const ReadingVector& v,
  std::vector<double>& coeff)
{
  int n = v.size();
  double x[n];
  double y[n];
  double c0, c1, cov00, cov01, cov11, sumsq;

  for(int i = 0; i < n; ++i)
  {  
    x[i] = v[i].measured;
    y[i] = v[i].ground_truth / x[i];
  }

  gsl_fit_linear(x, 1, y, 1, n, &c0, &c1, &cov00, &cov01, &cov11, &sumsq);

  coeff[0] = 0.0;
  coeff[1] = c0;
  coeff[2] = c1;
}

void DepthCalibrator::linearFit(
  const ReadingVector& v,
  std::vector<double>& coeff)
{
  int n = v.size();
  double x[n];
  double y[n];
  double c0, c1, cov00, cov01, cov11, sumsq;

  for(int i = 0; i < n; ++i)
  {  
    x[i] = v[i].measured;
    y[i] = v[i].ground_truth;
  }

  gsl_fit_linear(x, 1, y, 1, n, &c0, &c1, &cov00, &cov01, &cov11, &sumsq);

  coeff[0] = c0;
  coeff[1] = c1;
  coeff[2] = 0.0;
}

void DepthCalibrator::quadraticFit(
  const ReadingVector& v,
  std::vector<double>& coeff)
{
  int degree = 3;
  int obs = v.size();
  gsl_multifit_linear_workspace *ws;
  gsl_matrix *cov, *X;
  gsl_vector *y, *c;
  double chisq;
 
  X = gsl_matrix_alloc(obs, degree);
  y = gsl_vector_alloc(obs);
  c = gsl_vector_alloc(degree);
  cov = gsl_matrix_alloc(degree, degree);
 
  for(int i=0; i < obs; i++) 
  {
    gsl_matrix_set(X, i, 0, 1.0);
    for(int j = 0; j < degree; j++) 
      gsl_matrix_set(X, i, j, pow(v[i].measured, j));

    gsl_vector_set(y, i, v[i].ground_truth);
  }
 
  ws = gsl_multifit_linear_alloc(obs, degree);
  gsl_multifit_linear(X, y, c, cov, &chisq, ws);
 
  // store coefficients
  coeff.resize(degree);
  for(int i = 0; i < degree; i++)
    coeff[i] = gsl_vector_get(c, i);
 
  gsl_multifit_linear_free(ws);
  gsl_matrix_free(X);
  gsl_matrix_free(cov);
  gsl_vector_free(y);
  gsl_vector_free(c);
}

void DepthCalibrator::testDepthCalibration()
{
  // load calirbration matrices
  ROS_INFO("Reading coefficients info...");
  
  ros::WallTime start;
  
  loadCameraParams();
  buildRectMaps();
  build3dCornerVector();
  
  cv::FileStorage fs_coeff(calib_warp_filename_,  cv::FileStorage::READ);
  
  cv::Mat coeff0, coeff1, coeff2;
  
  fs_coeff["c0"] >> coeff0;
  fs_coeff["c1"] >> coeff1;
  fs_coeff["c2"] >> coeff2;
    
  // load test image
  ROS_INFO("Loading test images...");
  cv::Mat depth_img, rgb_img;
  rgb_img   = cv::imread(rgb_test_filename_);
  depth_img = cv::imread(depth_test_filename_, -1);
  
  // ********************************************************

  cv::Mat depth_img_g, depth_img_m;
  processTrainingImagePair(
    9999, rgb_img, depth_img, depth_img_g, depth_img_m);
 
  // measured and unwarped
  cv::Mat depth_img_mu = depth_img_m.clone();
  unwarpDepthImage(depth_img_mu, coeff0, coeff1, coeff2, fit_mode_);
  
  double rms_warped   = getRMSError(depth_img_g, depth_img_m);
  double rms_unwarped = getRMSError(depth_img_g, depth_img_mu);

  printf("RMS (warped): %f RMS (unwarped): %f\n", 
    rms_warped, rms_unwarped);
  
  // ********************************************************
 
   // rectify
  start = ros::WallTime::now();
  cv::Mat rgb_img_rect, depth_img_rect; 
  cv::remap(rgb_img,   rgb_img_rect,   map_rgb_1_, map_rgb_2_, cv::INTER_LINEAR);
  cv::remap(depth_img, depth_img_rect, map_ir_1_,  map_ir_2_,  cv::INTER_NEAREST);
  ROS_INFO("Rectifying: %.1fms", getMsDuration(start));
  
  // reproject warped
  cv::Mat depth_img_warped = depth_img_rect.clone(); 
  cv::Mat depth_img_rect_warped_reg;
  buildRegisteredDepthImage(
    intr_rect_ir_, intr_rect_rgb_, ir2rgb_,
    depth_img_warped, depth_img_rect_warped_reg);
    
  // unwarp
  start = ros::WallTime::now();
  cv::Mat depth_img_unwarped = depth_img_rect.clone();
  unwarpDepthImage(depth_img_unwarped, coeff0, coeff1, coeff2, fit_mode_);
  ROS_INFO("Unwarping: %.1fms", getMsDuration(start));
  
  // reproject unwarped
  start = ros::WallTime::now();
  cv::Mat depth_img_rect_unwarped_reg;
  buildRegisteredDepthImage(
    intr_rect_ir_, intr_rect_rgb_, ir2rgb_,
    depth_img_unwarped, depth_img_rect_unwarped_reg);
  ROS_INFO("Reprojecting: %.1fms", getMsDuration(start));

  // build point clouds
  ROS_INFO("Building point clouds...");
  
  PointCloudT cloud_warped, cloud_unwarped;
    
  buildPointCloud(
    depth_img_rect_warped_reg, rgb_img_rect, intr_rect_rgb_, cloud_warped);
  buildPointCloud(
    depth_img_rect_unwarped_reg, rgb_img_rect, intr_rect_rgb_, cloud_unwarped);
  
  // save point clouds
  std::string w_cloud_filename = test_path_ + "/warped.pcd";
  std::string u_cloud_filename = test_path_ + "/unwarped.pcd";
  
  pcl::io::savePCDFileBinary<PointT>(w_cloud_filename, cloud_warped);
  pcl::io::savePCDFileBinary<PointT>(u_cloud_filename, cloud_unwarped); 

  ROS_INFO("Done.");
}

double DepthCalibrator::getRMSError(
  const cv::Mat& g,
  const cv::Mat& m)
{
  double s_error = 0.0;
  int count = 0;

  for (int v = 0; v < g.cols; ++v)
  for (int u = 0; u < g.rows; ++u)
  {
    uint8_t z_g = g.at<uint8_t>(v, u); 
    uint8_t z_m = m.at<uint8_t>(v, u); 
    
    if (z_g != 0 && z_m != 0)
    {
      count++;
      s_error += (z_g - z_m) * (z_g - z_m);
    }
  }
  
  double ms_error = s_error / (double) count;
  double rms_error = sqrt(ms_error);
  
  return rms_error;
}
  
} //namespace ccny_rgbd

