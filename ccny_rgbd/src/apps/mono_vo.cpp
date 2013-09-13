/* ======================================================================
 * mono_vo.cpp
 *       Final Project
 *
 *  Written by Carlos Jaramillo, Roberto Valenti and Ivan Dryanovski
 *  3D Computer Vision - CSc 83020 at CUNY GC - Prof. Stamos - (Fall 2012)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 * ======================================================================
 */
#include "ccny_rgbd/apps/mono_vo.h"

namespace ccny_rgbd {

MonocularVisualOdometry::MonocularVisualOdometry(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false),
  frame_count_(0)
{
  ROS_INFO("Starting Monocular Visual Odometry from a 3D Dense Model");
  
  // **** init parameters
  initParams();
  f2b_.setIdentity();
 
    tf::Transform freiburg1;
  freiburg1.setOrigin(tf::Vector3(-0.0552041697502, -0.415246917725, 1.66852389526));
  freiburg1.setRotation(tf::Quaternion(-0.466079328013,
                                        0.0598513320547,
                                       -0.123544403938,
                                        0.874027836116));
  tf::Transform freiburg2;
  freiburg2.setOrigin(tf::Vector3(-0.0112272525515, 0.0502554918469, -0.0574041954071));
  freiburg2.setRotation(tf::Quaternion(0.129908557896,
                                      -0.141388223098,
                                       0.681948549539,
                                       0.705747343414));
  
  f2b_ = freiburg1 * freiburg2;
  
  // **** publishers
  if(publish_cloud_model_)
  {
    pub_model_ = nh_private.advertise<PointCloudT>(
      "model_3D", 1);
  }

  odom_publisher_ = nh_.advertise<OdomMsg>(
    "odom", 1);

  if(readPointCloudFromPCDFile()==false)
    ROS_FATAL("The sky needs its point cloud to operate!");

  // **** subscribers
  image_transport::ImageTransport rgb_it(nh_);
  sub_rgb_.subscribe(rgb_it, topic_image_, 1);
  sub_info_.subscribe(nh_, topic_cam_info_, 1);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  int queue_size = 5;
  sync_.reset(new SynchronizerMonoVO(SyncPolicyMonoVO(queue_size), sub_rgb_, sub_info_));
  sync_->registerCallback(boost::bind(&MonocularVisualOdometry::imageCallback, this, _1, _2));
}

MonocularVisualOdometry::~MonocularVisualOdometry()
{
  ROS_INFO("Destroying Monocular Visual Odometry");
}

std::string MonocularVisualOdometry::formKeyframeName(int keyframe_number, int num_of_chars)
{
  std::stringstream ss;
  ss << keyframe_number;
  std::string keyframe_number_as_str = ss.str();
  while(keyframe_number_as_str.size() < (uint) num_of_chars)
    keyframe_number_as_str = "0" + keyframe_number_as_str;

  return keyframe_number_as_str;
}

void MonocularVisualOdometry::generateKeyframePaths(const std::string& keyframe_path, int keyframe_number, std::string& current_keyframe_path, std::string& next_keyframe_path)
{
  current_keyframe_path = keyframe_path + formKeyframeName(keyframe_number, 4);
  next_keyframe_path = keyframe_path + formKeyframeName(keyframe_number+1, 4);
  std::cout << "Current Frame: " << current_keyframe_path << std::endl;
  std::cout << "Next Frame: " << next_keyframe_path << std::endl;
}

void MonocularVisualOdometry::initParams()
{
  // PCD File
  if(!nh_private_.getParam("apps/mono_vo/PCD_filename", pcd_filename_))
    pcd_filename_ = "cloud.pcd";

  if (!nh_private_.getParam ("apps/mono_vo/path_to_keyframes", path_to_keyframes_))
    path_to_keyframes_ = "~/ros/Keyframes";
  if (!nh_private_.getParam ("apps/mono_vo/initial_keyframe_number", initial_keyframe_number_))
    initial_keyframe_number_ = 0;

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("base_frame", base_frame_))
    base_frame_ = "/camera_link";

  // FIXME: use a param for the frame name
  path_pub_ = nh_.advertise<nav_msgs::Path>("/mono_path", 5);

  if (!nh_private_.getParam ("apps/mono_vo/detector_type", detector_type_))
    detector_type_ = "SURF";
  if (!nh_private_.getParam ("apps/mono_vo/descriptor_type", descriptor_type_))
    descriptor_type_ = "SURF";
  if (!nh_private_.getParam ("apps/mono_vo/detector_threshold", detector_threshold_))
    detector_threshold_ = 400;
  if (!nh_private_.getParam ("apps/mono_vo/max_descriptor_space_distance", max_descriptor_space_distance_))
    max_descriptor_space_distance_ = 0.5;
  if (!nh_private_.getParam ("apps/mono_vo/image_width", image_width_))
    image_width_ = 320;
  if (!nh_private_.getParam ("apps/mono_vo/image_height", image_height_))
    image_height_ = 240;
  if (!nh_private_.getParam ("apps/mono_vo/virtual_image_width", virtual_image_width_))
    image_width_ = 400;
  if (!nh_private_.getParam ("apps/mono_vo/virtual_image_height", virtual_image_height_))
    image_height_ = 300;
  if (!nh_private_.getParam ("apps/mono_vo/virtual_image_blur", virtual_image_blur_))
    virtual_image_blur_ = 3;
  if (!nh_private_.getParam ("apps/mono_vo/virtual_image_fill", virtual_image_fill_))
    virtual_image_fill_ = 3;
  if (!nh_private_.getParam ("apps/mono_vo/min_inliers_count", min_inliers_count_))
    min_inliers_count_ = 70;
  if (!nh_private_.getParam ("apps/mono_vo/max_ransac_iterations", max_ransac_iterations_))
    max_ransac_iterations_ = 1000;
  if (!nh_private_.getParam ("apps/mono_vo/max_reproj_error", max_reproj_error_))
    max_reproj_error_ = 16;

  if (!nh_private_.getParam ("apps/mono_vo/publish_cloud_model", publish_cloud_model_))
    publish_cloud_model_ = false;
  if (!nh_private_.getParam ("apps/mono_vo/publish_virtual_img", publish_virtual_img_))
    publish_virtual_img_ = false;

  if (!nh_private_.getParam ("apps/mono_vo/topic_cam_info", topic_cam_info_))
    topic_cam_info_ = "/camera/rgb/camera_info";
  if (!nh_private_.getParam ("apps/mono_vo/topic_image", topic_image_))
    topic_image_ = "/camera/rgb/image_rect_color";
  if (!nh_private_.getParam ("apps/mono_vo/topic_virtual_image", topic_virtual_image_))
    topic_virtual_image_ = "/camera/rgb/virtual";
}

void MonocularVisualOdometry::getVirtualImageFromKeyframe(
  const PointCloudT& cloud, 
  const Matrix3f& intrinsic, 
  const tf::Transform& extrinsic_tf, 
  cv::Mat& virtual_rgb_img, 
  cv::Mat& virtual_depth_img)
{
  Matrix3f rmat;
  Vector3f tvec;
  tfToEigenRt(extrinsic_tf, rmat, tvec);

  cv::Mat rgb_img_projected;
  cv::Mat depth_img_projected;

  projectCloudToImage(cloud, rmat, tvec, intrinsic, image_width_, image_height_, rgb_img_projected, depth_img_projected);

  
  holeFilling2(rgb_img_projected, depth_img_projected, virtual_image_fill_, virtual_rgb_img, virtual_depth_img);

  //cv::medianBlur(rgb_img,rgb_img, 3);
  if (virtual_image_blur_ > 1)
    cv::GaussianBlur(virtual_rgb_img, virtual_rgb_img, cv::Size(virtual_image_blur_, virtual_image_blur_), 0);
}

bool MonocularVisualOdometry::readPointCloudFromPCDFile()
{
//  cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  model_ptr_ = PointCloudT::Ptr(new PointCloudT);

  if (pcl::io::loadPCDFile<PointT> (pcd_filename_, *model_ptr_) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return false;
  }
  model_ptr_->header.frame_id = fixed_frame_;
  model_ptr_->header.stamp = ros::Time::now();

  std::cout << "Loaded "
      << model_ptr_->width * model_ptr_->height
      << " data points from " << pcd_filename_ << " with header = " <<   model_ptr_->header.frame_id
      << std::endl;

  // ****** HACK FOR FREIBURG BAG
  tf::Transform freiburg1;
  freiburg1.setOrigin(tf::Vector3(-0.0552041697502, -0.415246917725, 1.66852389526));
  freiburg1.setRotation(tf::Quaternion(-0.466079328013,
                                        0.0598513320547,
                                       -0.123544403938,
                                        0.874027836116));
  tf::Transform freiburg2;
  freiburg2.setOrigin(tf::Vector3(-0.0112272525515, 0.0502554918469, -0.0574041954071));
  freiburg2.setRotation(tf::Quaternion(0.129908557896,
                                      -0.141388223098,
                                       0.681948549539,
                                       0.705747343414));
  freiburg1 * freiburg2;
      
  pcl_ros::transformPointCloud<PointT>(
    *model_ptr_, *model_ptr_, (freiburg1 * freiburg2));
  
  pub_model_.publish(*model_ptr_);
  
  return true;
}


void MonocularVisualOdometry::imageCallback(
  const sensor_msgs::ImageConstPtr& rgb_msg, 
  const sensor_msgs::CameraInfoConstPtr& info_msg)
{ 
  // **** initialize ***************************************************
  
  if (!initialized_)
  {
    ROS_INFO("RGB header = %s", rgb_msg->header.frame_id.c_str());
    initialized_ = getBaseToCameraTf(rgb_msg->header);
    init_time_ = rgb_msg->header.stamp;

    if (!initialized_) return;

    cam_model_.fromCameraInfo(info_msg);
    openCVRToEigenR(cam_model_.intrinsicMatrix(), intrinsic_matrix_);
    float scale_factor_intrinsic = 1.0;
    // Rescale intrinsice
    scale_factor_intrinsic =  (float) image_width_ / (float) cam_model_.width();
    printf("Scale factor = %f \n", scale_factor_intrinsic);
    intrinsic_matrix_(0,0) = scale_factor_intrinsic*intrinsic_matrix_(0,0); // fx
    intrinsic_matrix_(0,2) = scale_factor_intrinsic*intrinsic_matrix_(0,2); // cx
    intrinsic_matrix_(1,1) = scale_factor_intrinsic*intrinsic_matrix_(1,1); // fy
    intrinsic_matrix_(1,2) = scale_factor_intrinsic*intrinsic_matrix_(1,2); // cy

    printf("Initialization successful at Frame %d\n", frame_count_);
  }

  cv::Mat rgb_img = cv_bridge::toCvShare(rgb_msg)->image;

  // Process frame for position estimation
  estimatePose(model_ptr_, rgb_img);

  if(publish_cloud_model_ && (frame_count_ % 100 == 0)) // Don't publish too often because it slows things down
  {
    pub_model_.publish(*model_ptr_);
  }
 
  frame_count_++;
}

void MonocularVisualOdometry::estimatePose(
  const PointCloudT::Ptr& model_cloud, 
  const cv::Mat& mono_img)
{ 
  ros::WallTime start = ros::WallTime::now();
  
  // **** parameters **********************************************************
  
  bool draw_image_pair        = true;
  bool draw_candidate_matches = false;
  bool draw_inlier_matches    = true; 
  
  // **** get virtual image ***************************************************
  ros::WallTime start_proj = ros::WallTime::now();
  tf::Transform f2c = f2b_ * b2c_;  // transofrm fixed frame to camera frame
  cv::Mat virtual_img, virtual_depth_img;
  getVirtualImageFromKeyframe(*model_cloud, intrinsic_matrix_, f2c.inverse(), virtual_img, virtual_depth_img); 
  ros::WallTime end_proj = ros::WallTime::now();
  // Resize monocular image:
  cv::Mat mono_img_resized;
  cv::resize(mono_img, mono_img_resized, cv::Size(image_width_, image_height_) );

  if(draw_image_pair)
  {
    cv::namedWindow("Virtual Image", 0);
    cv::namedWindow("Monocular Image", 0);
    cv::imshow("Virtual Image", virtual_img);
    cv::imshow("Monocular Image", mono_img_resized);
    cv::waitKey(1);
  }
  

  // **** get motion estimation *********************************************

  // **** Feature detection
  ros::WallTime start_detect = ros::WallTime::now();

  // mask for virtual image - masks empty areas
  cv::Mat virtual_img_mask;
  virtual_depth_img.convertTo(virtual_img_mask, CV_8U);
  
  // feature detection
  cv::SurfFeatureDetector feature_detector(detector_threshold_);
  std::vector<cv::KeyPoint> keypoints_virtual, keypoints_mono;
  feature_detector.detect(virtual_img, keypoints_virtual, virtual_img_mask);
  feature_detector.detect(mono_img_resized, keypoints_mono);

  // descriptor extraction
  cv::SurfDescriptorExtractor descriptor_extractor; 
  cv::Mat descriptors_virtual, descriptors_mono;
  descriptor_extractor.compute(virtual_img, keypoints_virtual, descriptors_virtual);
  descriptor_extractor.compute(mono_img_resized, keypoints_mono, descriptors_mono);
  
  ros::WallTime end_detect = ros::WallTime::now();  
  
  // **** Feature matching
  ros::WallTime start_match = ros::WallTime::now();

  // build candidate matches
  cv::FlannBasedMatcher matcher_flann;
  std::vector<cv::DMatch> all_matches;
  
  // match QUERY (2d) onto TRAIN (3d)
  matcher_flann.match(descriptors_mono, descriptors_virtual, all_matches);

  // create vector of 2D and 3D point correspondences for PnP
  std::vector<cv::Point2f> corr_2D_points_vector;
  std::vector<cv::Point3f> corr_3D_points_vector;

  Matrix3f intr_inv = intrinsic_matrix_.inverse();
  
  // remove bad matches - too far away in descriptor space,
  std::vector<cv::DMatch> candidate_matches;
  for (unsigned int m_idx = 0; m_idx < all_matches.size(); ++m_idx)
  {
    const cv::DMatch& match = all_matches[m_idx];

    if (match.distance < max_descriptor_space_distance_)
    {
      int idx_query = match.queryIdx;
      int idx_train = match.trainIdx;
      
      const cv::Point2f& point2f_virtual = keypoints_virtual[idx_train].pt;
      const cv::Point2f& point2f_mono    = keypoints_mono[idx_query].pt;
      
      int virtual_u = (int)(point2f_virtual.x);
      int virtual_v = (int)(point2f_virtual.y);      
      
      uint16_t depth = virtual_depth_img.at<uint16_t>(virtual_v, virtual_u);

      if(depth > 0)
      {
        float z = depth / 1000.0;
        
        candidate_matches.push_back(match);
        corr_2D_points_vector.push_back(point2f_mono);

        Vector3f p;
        p(0,0) = point2f_virtual.x * z;
        p(1,0) = point2f_virtual.y * z;
        p(2,0) = z;

        Vector3f P = intr_inv * p;

        cv::Point3d point3d_virtual;
        point3d_virtual.x = P(0,0);
        point3d_virtual.y = P(1,0);
        point3d_virtual.z = P(2,0);
        corr_3D_points_vector.push_back(point3d_virtual);
      }
    }
  }

  if(draw_candidate_matches)
  {
    cv::Mat virtual_img_copy = virtual_img.clone();
    cv::Mat mono_img_copy    = mono_img_resized.clone();

    cv::Mat matches_result_img;
    cv::drawMatches(
      mono_img_copy, keypoints_mono,        // Query image and its keypoints
      virtual_img_copy, keypoints_virtual,  // Train image and its keypoints
      candidate_matches, 
      matches_result_img);
    
    cv::namedWindow("Matches FLANN", 0); 
    cv::imshow("Matches FLANN", matches_result_img);
  }

  // Fix max inliers count to be reasonable within number of descriptors
  int number_of_candidate_matches = candidate_matches.size();
  if(number_of_candidate_matches < min_inliers_count_)
    min_inliers_count_ = number_of_candidate_matches; // update minimum

  ros::WallTime end_match = ros::WallTime::now();
    
  // **** transformation computation with PnP
  ros::WallTime start_pnp = ros::WallTime::now();
  
  cv::Mat M; // The intrinsic matrix
  cv3x3FromEigen(intrinsic_matrix_, M);

  cv::Mat rvec, rmat;
  cv::Mat tvec;

  std::vector<int> inliers_indices;

  cv::solvePnPRansac(
    corr_3D_points_vector, 
    corr_2D_points_vector, M, cv::Mat(), 
    rvec, tvec, false,
    max_ransac_iterations_, max_reproj_error_, 
    min_inliers_count_, inliers_indices,
    CV_ITERATIVE);
  
  if(draw_inlier_matches)
  {
    std::vector<cv::DMatch> inliers_matches;

    for (unsigned int m_idx = 0; m_idx < inliers_indices.size(); ++m_idx)
    {
      cv::DMatch match = candidate_matches[inliers_indices[m_idx]];
      inliers_matches.push_back(match);
    }
    
    cv::Mat virtual_img_copy = virtual_img.clone();
    cv::Mat mono_img_copy    = mono_img_resized.clone();

    cv::Mat matches_result_img;
    cv::drawMatches(
        mono_img_copy,    keypoints_mono,     // Query image and its keypoints
        virtual_img_copy, keypoints_virtual,  // Train image and its keypoints
        inliers_matches, 
        matches_result_img);

    cv::namedWindow("Inlier matches", 0);
    cv::imshow("Inlier matches", matches_result_img);
    cv::waitKey(0);
  }

  cv::Rodrigues(rvec, rmat);
  tf::Transform pnp_extr;
  openCVRtToTf(rmat, tvec, pnp_extr);
  tf::Transform transform_est = pnp_extr.inverse();
  
  ros::WallTime end_pnp = ros::WallTime::now();
   
  // **** update poses ******************************************************

  tf::Transform f2c_new = f2c * transform_est; 
  f2b_ = f2c_new * b2c_.inverse(); 
  publishTransform(f2b_, fixed_frame_, base_frame_);
  
  ros::WallTime end = ros::WallTime::now();
  
  // **** profiling *********************************************************
  
  double dur_proj   = 1000.0 * (end_proj   - start_proj).toSec();
  double dur_detect = 1000.0 * (end_detect - start_detect).toSec();
  double dur_match  = 1000.0 * (end_match  - start_match).toSec();
  double dur_pnp    = 1000.0 * (end_pnp    - start_pnp).toSec();
  double dur        = 1000.0 * (end        - start).toSec();
  
  printf("[%d] Proj %.1f Det[%d][%d]: %.1f Match[%d][%d][%d]: %.1f PnP: %.1f TOTAL: %.1f\n",
    frame_count_, dur_proj, 
    (int)keypoints_virtual.size(), (int)keypoints_mono.size(), dur_detect,
    (int)all_matches.size(), (int)candidate_matches.size(), (int)inliers_indices.size(), 
    dur_match, dur_pnp, dur);
}

void MonocularVisualOdometry::publishTransform(const tf::Transform &source2target_transform, const std::string& source_frame_id, const std::string& target_frame_id)
{
  ros::Time current_time = ros::Time::now();
  tf::StampedTransform transform_msg(
        source2target_transform, current_time, source_frame_id, target_frame_id);
  tf_broadcaster_.sendTransform (transform_msg);

  OdomMsg odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = source_frame_id;
  tf::poseTFToMsg(source2target_transform, odom.pose.pose);
  odom_publisher_.publish(odom);

  publishPath(odom.header);
}

void MonocularVisualOdometry::publishPath(const std_msgs::Header& header)
{
  path_msg_.header.stamp = header.stamp;
  path_msg_.header.frame_id = fixed_frame_;

  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.stamp = header.stamp;
  pose_stamped.header.frame_id = fixed_frame_;
  tf::poseTFToMsg(f2b_, pose_stamped.pose);

  path_msg_.poses.push_back(pose_stamped);
  path_pub_.publish(path_msg_);
}

bool MonocularVisualOdometry::getBaseToCameraTf(const std_msgs::Header& header)
{
  tf::StampedTransform tf_m;

  ROS_INFO("Transforming Base (%s) to Camera (%s)", base_frame_.c_str(), header.frame_id.c_str());
  try
  {
    tf_listener_.waitForTransform(
      base_frame_, header.frame_id, header.stamp, ros::Duration(1.0));
    tf_listener_.lookupTransform (
      base_frame_, header.frame_id, header.stamp, tf_m);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("Base to camera transform unavailable: %s", ex.what());
    return false;
  }

  b2c_ = tf_m;

  ROS_INFO("Successfully transformed Base to Camera");
  return true;
}

} //namespace ccny_rgbd
