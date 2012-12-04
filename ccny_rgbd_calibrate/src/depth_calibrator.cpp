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
  calib_rgb_filename_  = path_ + "rgb.yml";
  calib_ir_filename_   = path_ + "depth.yml";
  calib_extr_filename_ = path_ + "extr.yml";
   
  //calibrate();
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
  std::string rgb_filename   = path_ + "train_depth/rgb/"   + ss_filename.str();
  std::string depth_filename = path_ + "train_depth/depth/" + ss_filename.str();

  //ROS_INFO("Trying images \n\t%s\n\t%s", rgb_filename.c_str(), depth_filename.c_str());
  
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

  // **** load intrinsics and distortion coefficients
  ROS_INFO("Reading camera info...");
  cv::FileStorage fs_rgb (calib_rgb_filename_,  cv::FileStorage::READ);
  cv::FileStorage fs_ir  (calib_ir_filename_,   cv::FileStorage::READ);
  cv::FileStorage fs_extr(calib_extr_filename_, cv::FileStorage::READ);
    
  fs_rgb["camera_matrix"]           >> intr_rgb_;
  fs_rgb["distortion_coefficients"] >> dist_rgb_;
  fs_ir ["camera_matrix"]           >> intr_ir_;
  fs_ir ["distortion_coefficients"] >> dist_ir_;
  
  fs_extr["rgb2ir"] >> rgb2ir_;
  
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
     
  std::cout << "intr_ir_" << intr_ir_ << std::endl;
  std::cout << "intr_rect_ir_" << intr_rect_ir_ << std::endl;    
  
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
  // TODO: parameters
  int w = 7; // windows size = 2w+1
  
  if (!loadCameraParams()) return;
  buildRectMaps();
  build3dCornerVector();
  
  cv::Mat count_img = cv::Mat::zeros(480, 640, CV_8UC1);
  
  std::vector<ReadingVector> readings;
  readings.resize(640*480);
  
  std::vector<DoubleVector> coeffs;
  coeffs.resize(640*480);
  
  cv::Mat c0(480, 640, CV_64FC1);
  cv::Mat c1(480, 640, CV_64FC1);
  cv::Mat c2(480, 640, CV_64FC1);
  
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
    
    cv::imshow("count", count_img);
    cv::waitKey(1);
       
    // increment image index
    img_idx++;
  }
  
  // perform fitting
  ROS_INFO("Fitting...");
 
  for(int u = 0; u < 640; ++u)
  for(int v = 0; v < 480; ++v) 
  {
    int idx = v * 640 + u; 
     
    // acumulate data points from neighbors
    ReadingVector vec;
    for(int uu = u-w; uu <= u+w; ++uu)
    for(int vv = v-w; vv <= v+w; ++vv)
    {    
      if (uu < 0 || uu >=640 || vv < 0 || vv >= 480) continue;
           
      int idx_n = vv * 640 + uu; 
      
      const ReadingVector& vec_n = readings[idx_n];
      vec.insert(vec.end(), vec_n.begin(), vec_n.end());
    }
    
    //const ReadingVector& vec = readings[idx];
    DoubleVector& coeff = coeffs[idx]; 
    
    if (vec.size() >= 3)
    {
      // 3 or more points - do quadratic fit
      polynomialFit(3, vec, coeff);
    }
    else
    {
      // two or less points - just use constant model
      // TODO: linear model for 2 pts?
      // TODO: maybe interpolate from neighbors?
      coeff.push_back(0);
      coeff.push_back(1);
      coeff.push_back(0);
    }
    
    // write to image
    c0.at<double>(v, u) = coeff[0];
    c1.at<double>(v, u) = coeff[1];
    c2.at<double>(v, u) = coeff[2];
  }
  
  // **** write to file filename
  ROS_INFO("Writing to file...");
  std::ofstream datafile;
  std::string data_filename = path_ + "data.txt";
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
  
  cv::FileStorage fs(path_ + "warp.yml", cv::FileStorage::WRITE);
  fs << "c0" << c0;
  fs << "c1" << c1;
  fs << "c2" << c2;
   
  cv::waitKey(0);
}

bool DepthCalibrator::processTrainingImagePair(
  int img_idx,
  const cv::Mat& rgb_img,
  const cv::Mat& depth_img,
  cv::Mat& depth_img_g,
  cv::Mat& depth_img_m)
{
  ROS_INFO("[%d] Processing image pair...", img_idx);
  
  depth_img_g = cv::Mat(rgb_img.size(), CV_16UC1);
  depth_img_m = cv::Mat(rgb_img.size(), CV_16UC1);
  
  // rectify
  cv::Mat rgb_img_rect, depth_img_rect; // rectified images 
  cv::remap(rgb_img,   rgb_img_rect,   map_rgb_1_, map_rgb_2_, cv::INTER_LINEAR);
  cv::remap(depth_img, depth_img_rect, map_ir_1_,  map_ir_2_,  cv::INTER_NEAREST);

  // detect corners

  std::vector<cv::Point2f> corners_2d_rgb;

  bool corner_result_rgb = getCorners(rgb_img_rect, patternsize_, corners_2d_rgb);
  if (!corner_result_rgb)
  {
    ROS_WARN("[%d] Corner detection failed. Skipping image pair", img_idx);
    return false;
  }
  
  // show images with corners
  showCornersImage(rgb_img_rect, patternsize_, corners_2d_rgb, 
                   corner_result_rgb, "RGB Corners");
  cv::waitKey(1);
  
  // get pose from RGB camera to checkerboard
  cv::Mat rvec, tvec;
  bool pnp_result = solvePnP(corners_3d_, corners_2d_rgb, 
    intr_rect_rgb_, cv::Mat(), rvec, tvec);

  if (!pnp_result)
  {
    ROS_WARN("[%d] PnP failed. Skipping image pair.", img_idx);
    return false;
  }
  
  cv::Mat rgb_to_board;
  matrixFromRvecTvec(rvec, tvec, rgb_to_board);
  
  // **** reproject
  cv::Mat depth_img_rect_reg;
  
  buildRegisteredDepthImage(
    intr_rect_ir_, intr_rect_rgb_, rgb2ir_,
    depth_img_rect, depth_img_rect_reg);
  
  // **** visualize 
  //showBlendedImage(depth_img_rect,     rgb_img_rect, "Unregistered");
  //showBlendedImage(depth_img_rect_reg, rgb_img_rect, "Registered");  
  //cv::waitKey(0);
     
  // **** filter depth image by contours
  Point2fVector vertices;
  getCheckerBoardPolygon(corners_2d_rgb, n_rows_, n_cols_, vertices);
  
  for (int u = 0; u < depth_img_rect_reg.cols; ++u)  
  for (int v = 0; v < depth_img_rect_reg.rows; ++v)
  {
    float dist = cv::pointPolygonTest(vertices, cv::Point2f(u,v), true);
    if (dist <= 0)
      depth_img_m.at<uint16_t>(v, u) = 0;
    else
      depth_img_m.at<uint16_t>(v, u) = depth_img_rect_reg.at<uint16_t>(v, u);
  }
  
  // **** build measured point cloud  
  PointCloudT m_cloud;
  buildPointCloud(depth_img_m, rgb_img_rect, intr_rect_rgb_, m_cloud);
  
  // **** build ground-truth depth image from checkerboard
 
  buildCheckerboardDepthImage(
    corners_3d_, vertices, depth_img_g.rows, depth_img_g.cols,
    rvec, tvec, intr_rect_rgb_, depth_img_g);
    
  // **** build ground-truth point cloud  
  PointCloudT g_cloud;
  buildPointCloud(depth_img_g, rgb_img_rect, intr_rect_rgb_, g_cloud);
  
/*
  for (unsigned int cn_idx = 0; cn_idx < corners_3d_.size(); ++cn_idx)
  {
    const cv::Point3f& corner = corners_3d_[cn_idx];
    cv::Mat corner_4d = cv::Mat(4,1, CV_64FC1);
    corner_4d.at<double>(0,0) = corner.x;
    corner_4d.at<double>(1,0) = corner.y;
    corner_4d.at<double>(2,0) = corner.z;
    corner_4d.at<double>(3,0) = 1.0;  
    
    cv::Mat corner_tf = rgb_to_board * corner_4d;
    
    PointT pt;
    pt.x = corner_tf.at<double>(0,0); 
    pt.y = corner_tf.at<double>(1,0);
    pt.z = corner_tf.at<double>(2,0);
                  
    g_cloud.push_back(pt);
  }
  */
  
  // **** save clouds
  std::stringstream ss_filename;
  ss_filename << std::setw(4) << std::setfill('0') << img_idx << ".pcd";
  
  std::string m_cloud_filename = path_  + "m_" +  ss_filename.str();
  std::string g_cloud_filename = path_  + "g_" +  ss_filename.str();
  
  pcl::io::savePCDFileBinary<PointT>(m_cloud_filename, m_cloud);
  pcl::io::savePCDFileBinary<PointT>(g_cloud_filename, g_cloud); 
  
  return true;
}

/* obs: how many data points
 * degree: degree of polynomial
 * dx: array of x data
 * dy: array of y data
 */

bool DepthCalibrator::polynomialFit(
  int degree, 
  const ReadingVector& v,
  std::vector<double>& coeff)
{
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
  return true;
}

void DepthCalibrator::testDepthCalibration()
{
  // load calirbration matrices
  ROS_INFO("Reading coefficients info...");
  
  ros::WallTime start;
  
  loadCameraParams();
  buildRectMaps();
  build3dCornerVector();
  
  cv::FileStorage fs_coeff(path_ + "warp.yml",  cv::FileStorage::READ);
  
  cv::Mat coeff0, coeff1, coeff2;
  
  fs_coeff["c0"] >> coeff0;
  fs_coeff["c1"] >> coeff1;
  fs_coeff["c2"] >> coeff2;
    
  // load test image
  ROS_INFO("Loading test images...");
  cv::Mat depth_img, rgb_img;
  rgb_img   = cv::imread(path_ + "train_depth/rgb/0100.png");
  depth_img = cv::imread(path_ + "train_depth/depth/0100.png", -1);
  
  // rectify
  start = ros::WallTime::now();
  cv::Mat rgb_img_rect, depth_img_rect; // rectified images 
  cv::remap(rgb_img,   rgb_img_rect,   map_rgb_1_, map_rgb_2_, cv::INTER_LINEAR);
  cv::remap(depth_img, depth_img_rect, map_ir_1_,  map_ir_2_,  cv::INTER_NEAREST);
  printf("Rectifying: %.1fms\n", (ros::WallTime::now() - start).toSec() * 1000.0);
  
  // reproject
  start = ros::WallTime::now();
  cv::Mat depth_img_rect_reg;
  buildRegisteredDepthImage(intr_rect_ir_, intr_rect_rgb_, rgb2ir_,
                            depth_img_rect, depth_img_rect_reg);
  printf("Reprojecting: %.1fms\n", (ros::WallTime::now() - start).toSec() * 1000.0);
    
  // uwarp
  cv::Mat depth_img_warped   = depth_img_rect_reg.clone();
  cv::Mat depth_img_unwarped = depth_img_rect_reg.clone(); 
  
  start = ros::WallTime::now();
  unwarpDepthImage(depth_img_unwarped, coeff0, coeff1, coeff2);
  printf("Unwarping: %.1fms\n", (ros::WallTime::now() - start).toSec() * 1000.0);
  
  // build point clouds
  ROS_INFO("building point clouds...");
  
  PointCloudT cloud_warped, cloud_unwarped;
    
  buildPointCloud(
    depth_img_warped, rgb_img_rect, intr_rect_rgb_, cloud_warped);
  buildPointCloud(
    depth_img_unwarped, rgb_img_rect, intr_rect_rgb_, cloud_unwarped);
  
  // save point clouds
  std::string w_cloud_filename = path_  + "warped.pcd";
  std::string u_cloud_filename = path_  + "unwarped.pcd";
  
  pcl::io::savePCDFileBinary<PointT>(w_cloud_filename, cloud_warped);
  pcl::io::savePCDFileBinary<PointT>(u_cloud_filename, cloud_unwarped); 
}

} //namespace ccny_rgbd
