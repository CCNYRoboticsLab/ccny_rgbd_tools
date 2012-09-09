#include "ccny_rgbd/structures/rgbd_frame.h"

namespace ccny_rgbd
{

RGBDFrame::RGBDFrame():
  keypoints_computed(false),
  descriptors_computed(false)
{

}

RGBDFrame::RGBDFrame(const sensor_msgs::ImageConstPtr& rgb_msg,
                     const sensor_msgs::ImageConstPtr& depth_msg,
                     const sensor_msgs::CameraInfoConstPtr& info_msg):
  keypoints_computed(false),
  descriptors_computed(false)  
{
  // TODO: Share vs copy?
  cv_ptr_rgb_   = cv_bridge::toCvCopy(rgb_msg);
  cv_ptr_depth_ = cv_bridge::toCvCopy(depth_msg);

  // create camera model
  model_.fromCameraInfo(info_msg);

  // record header - frame is from RGB camera
  header = rgb_msg->header;
}

// input - z [meters]
// output - std. dev of z  [meters] according to calibration paper
double RGBDFrame::getStdDevZ(double z)
{
  double z_model_constant = 0.001425;
  return z_model_constant * z * z;
}

// input - z [meters]
// output - variance of z [meters] according to calibration paper
double RGBDFrame::getVarZ(double z)
{
  double std_dev_z = getStdDevZ(z);
  return std_dev_z * std_dev_z;
}

void RGBDFrame::getGaussianDistribution(
  int u, int v, double& z_mean, double& z_var)
{
  // get raw z value (in mm)
  uint16_t z_raw = cv_ptr_depth_->image.at<uint16_t>(v, u);

  // z [meters]
  z_mean = z_raw * 0.001;

  // var_z [meters]
  z_var = getVarZ(z_mean);
}

void RGBDFrame::getGaussianMixtureDistribution(
  int u, int v, double& z_mean, double& z_var)
{
  int w = 1;

  int u_start = std::max(u - w, 0);
  int v_start = std::max(v - w, 0);
  int u_end   = std::min(u + w, cv_ptr_depth_->image.cols - 1);
  int v_end   = std::min(v + w, cv_ptr_depth_->image.rows - 1);

  // iterate accross window - find mean
  double weight_sum = 0.0;
  double mean_sum   = 0.0;
  double alpha_sum  = 0.0;

  for (int uu = u_start; uu <= u_end; ++uu)
  for (int vv = v_start; vv <= v_end; ++vv)
  {
    uint16_t z_neighbor_raw = cv_ptr_depth_->image.at<uint16_t>(vv, uu);
 
    if (z_neighbor_raw != 0)
    {
      double z_neighbor = z_neighbor_raw * 0.001;

      // determine and aggregate weight
      double weight;
      if       (u==uu && v==vv) weight = 4.0;
      else if  (u==uu || v==vv) weight = 2.0;
      else                      weight = 1.0; 
      weight_sum += weight;

      // aggregate mean
      mean_sum += weight * z_neighbor;

      // aggregate var
      double var_z_neighbor = getVarZ(z_neighbor);
      alpha_sum += weight * (var_z_neighbor + z_neighbor * z_neighbor);
    }
  }

  z_mean = mean_sum  / weight_sum;
  z_var  = alpha_sum / weight_sum - z_mean * z_mean;
}

void RGBDFrame::computeDistributions()
{
  // TODO: these should be a parameter
  double max_var_z = 0.05 * 0.05; // maximum allowed z variance

  double s_u = 1.0;            // uncertainty in pixels
  double s_v = 1.0;            // uncertainty in pixels

  // center point
  double cx = model_.cx();
  double cy = model_.cy();

  // focus length
  double fx = model_.fx();
  double fy = model_.fy();

  // precompute for convenience
  double var_u = s_u * s_u;
  double var_v = s_v * s_v;
  double fx2 = fx*fx;
  double fy2 = fy*fy;

  // allocate space
  kp_valid.resize(keypoints.size());
  kp_mean.resize(keypoints.size());
  kp_covariance.resize(keypoints.size());

  for (unsigned int kp_idx = 0; kp_idx < keypoints.size(); ++kp_idx)
  {
    // calculate pixel coordinates
    double u = keypoints[kp_idx].pt.x;
    double v = keypoints[kp_idx].pt.y;  

    // get raw z value
    uint16_t z_raw = cv_ptr_depth_->image.at<uint16_t>((int)v, (int)u);

    // skip bad values  
    if (z_raw == 0)
    {
      kp_valid[kp_idx] = false;
      continue;
    }
    //kp_valid[kp_idx] = true;
  
    // get z: mean and variance
    double z, var_z;
    //getGaussianDistribution(u, v, z, var_z);
    getGaussianMixtureDistribution(u, v, z, var_z);

    // skip bad values  
    if (var_z > max_var_z)
    {
      kp_valid[kp_idx] = false;
      continue;
    }
    kp_valid[kp_idx] = true;

    // precompute for convenience
    double z_2  = z * z;
    double umcx = u - cx;
    double vmcy = v - cy;

    // calculate x and y
    double x = z * umcx / fx;
    double y = z * vmcy / fy;
  
    // calculate covariances
    double s_xz = var_z * umcx / fx;
    double s_yz = var_z * vmcy / fy;

    double s_xx = (var_z * umcx * umcx + var_u * (z_2 + var_z))/fx2;
    double s_yy = (var_z * vmcy * vmcy + var_v * (z_2 + var_z))/fy2;

    double s_xy = umcx * vmcy * var_z / (fx * fy);
    double s_yx = s_xy;

    double s_zz = var_z; 

    double s_zy = s_yz;
    double s_zx = s_xz;
   
    // fill out mean matrix
    kp_mean[kp_idx] = cv::Mat(3, 1, CV_64F);
    kp_mean[kp_idx].at<double>(0,0) = x;
    kp_mean[kp_idx].at<double>(1,0) = y;
    kp_mean[kp_idx].at<double>(2,0) = z;

    // fill out covariance matrix
    kp_covariance[kp_idx] = cv::Mat(3, 3, CV_64F);

    kp_covariance[kp_idx].at<double>(0,0) = s_xx; // xx
    kp_covariance[kp_idx].at<double>(0,1) = s_xy; // xy
    kp_covariance[kp_idx].at<double>(0,2) = s_xz; // xz

    kp_covariance[kp_idx].at<double>(1,0) = s_yx; // xy
    kp_covariance[kp_idx].at<double>(1,1) = s_yy; // yy
    kp_covariance[kp_idx].at<double>(1,2) = s_yz; // yz

    kp_covariance[kp_idx].at<double>(2,0) = s_zx; // xz-
    kp_covariance[kp_idx].at<double>(2,1) = s_zy; // yz
    kp_covariance[kp_idx].at<double>(2,2) = s_zz; // zz

/*
    printf("-----------------------\n");
    printf("%.2f, %.2f, %.2f\n", x, y, z);
    printf("-----------------------\n");
    printf("%.2f, %.2f, %.2f\n", s_xx*1000.0, s_xy*1000.0, s_xz*1000.0);
    printf("%.2f, %.2f, %.2f\n", s_yx*1000.0, s_yy*1000.0, s_yz*1000.0);
    printf("%.2f, %.2f, %.2f\n", s_zx*1000.0, s_zy*1000.0, s_zz*1000.0);
    printf("%.2f, %.2f, %.2f\n", sqrt(s_xx)*1000.0, sqrt(s_yy)*1000.0, sqrt(s_zz)*1000.0);
*/
    // ****** distort FIXME: remove *********************************
    double factor = 0.02;
    double factor_s = 1.0 + factor * (std::abs(umcx) / 160) + factor * (std::abs(vmcy) / 120);
    kp_mean[kp_idx].at<double>(2,0) = z * factor_s;

    // **************************************************************
  }


}

void RGBDFrame::constructFeatureCloud(float max_range, bool filter)
{
  for (unsigned int kp_idx = 0; kp_idx < keypoints.size(); ++kp_idx)
  {
    if (!kp_valid[kp_idx]) continue;

    double z = kp_mean[kp_idx].at<double>(2,0);
   
    // TODO: covariance filtering instead of depth?
    if (z <= max_range)
    {
      PointFeature p;

      p.x = kp_mean[kp_idx].at<double>(0,0);
      p.y = kp_mean[kp_idx].at<double>(1,0);
      p.z = z;

      features.points.push_back(p);
    }
  }

  features.header = header;
  features.height = 1;
  features.width = features.points.size();
  features.is_dense = true;
}

void RGBDFrame::constructCloudsFromInliers(
  const std::vector<cv::DMatch>& inlier_matches,
  const RGBDFrame& frame_src, 
  const RGBDFrame& frame_dst, 
  PointCloudT::Ptr& cloud_src,
  PointCloudT::Ptr& cloud_dst)
{
  // create 3D point clouds
  for (unsigned int i = 0; i < inlier_matches.size(); ++i)
  {
    int src_idx = inlier_matches[i].queryIdx;
    int dst_idx = inlier_matches[i].trainIdx;

    // check for invalid measurements
    if (frame_src.kp_valid[src_idx] && frame_dst.kp_valid[dst_idx])
    {
      PointT p_src, p_dst;
      cv::Mat src_mat = frame_src.kp_mean[src_idx];
      cv::Mat dst_mat = frame_dst.kp_mean[dst_idx];

      // fill in XYZ for src
      p_src.x = src_mat.at<double>(0,0);
      p_src.y = src_mat.at<double>(1,0);
      p_src.z = src_mat.at<double>(2,0);

      // fill in XYZ for dst
      p_dst.x = dst_mat.at<double>(0,0);
      p_dst.y = dst_mat.at<double>(1,0);
      p_dst.z = dst_mat.at<double>(2,0);

      cloud_src->push_back(p_src);
      cloud_dst->push_back(p_dst);
    }
  }

  cloud_src->height = 1;
  cloud_src->width = cloud_src->points.size();
  cloud_src->is_dense = true;

  cloud_dst->height = 1;
  cloud_dst->width = cloud_dst->points.size();
  cloud_dst->is_dense = true;
}

bool RGBDFrame::ransacMatchingOverlap(
  RGBDFrame& frame_src, RGBDFrame& frame_dst, 
  tf::Transform& transform, float matching_distance, 
  float eps_reproj, float inlier_threshold,
  PointCloudT::Ptr cloud_src, PointCloudT::Ptr cloud_dst)
{
  bool show = true;
  bool save = false;

  // **** params

  bool use_icp_refinement_sparse = true;

  // **** if needed, detect keypoints
    
  if (!frame_src.keypoints_computed)
  {
    cv::SurfFeatureDetector detector;
    detector.detect(*(frame_src.getRGBImage()), frame_src.keypoints);
    frame_src.keypoints_computed = true;
  }
  if (!frame_dst.keypoints_computed)
  {
    cv::SurfFeatureDetector detector;
    detector.detect(*(frame_dst.getRGBImage()), frame_dst.keypoints);
    frame_dst.keypoints_computed = true;
  }

  // **** if needed, extract descriptors

  //if (!frame_src.descriptors_computed)
  if(1)  
  {
    ROS_WARN("no descriptors present, computing SURF descriptors");
    cv::SurfDescriptorExtractor extractor;
    extractor.compute(*(frame_src.getRGBImage()), frame_src.keypoints, frame_src.descriptors);
    frame_src.descriptors_computed = true;
  }
  if(1)
  //if (!frame_dst.descriptors_computed)
  {
    ROS_WARN("no descriptors present, computing SURF descriptors");
    cv::SurfDescriptorExtractor extractor;
    extractor.compute(*(frame_dst.getRGBImage()), frame_dst.keypoints, frame_dst.descriptors);
    frame_dst.descriptors_computed = true;
  }

  // **** match the descriptors
  //cv::FlannBasedMatcher matcher;
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector<cv::DMatch> matches;
  matcher.match(frame_src.descriptors, frame_dst.descriptors, matches);

  std::vector<cv::DMatch> good_matches;

  for (unsigned int i = 0; i < matches.size(); ++i)
  {
    if (matches[i].distance < matching_distance)
      good_matches.push_back(matches[i]);
  }

  // **** create vectors of feature points from correspondences

  std::vector<cv::Point2f> h_src_pts;
  std::vector<cv::Point2f> h_dst_pts;

  for(unsigned int i = 0; i < good_matches.size(); ++i)
  {
    int q_idx = good_matches[i].queryIdx;
    int t_idx = good_matches[i].trainIdx; 

    h_src_pts.push_back(frame_src.keypoints[q_idx].pt);
    h_dst_pts.push_back(frame_dst.keypoints[t_idx].pt);
  }

  // **** Find inliers using ransac

  bool ransac_overlap;
  double inlier_ratio;
  std::vector<cv::DMatch> inlier_matches;

  cv::Mat status;
  cv::Mat h = cv::findHomography(h_src_pts, h_dst_pts, CV_RANSAC, eps_reproj, status);

  for (unsigned int m = 0; m < good_matches.size(); m++) 
  { 
    if (status.at<char>(m, 0) == 1)
      inlier_matches.push_back(good_matches[m]);
  }

  // drawing & save the raw matches

  if (show)
  {
    cv::Mat img_matches;
    cv::drawMatches(*(frame_src.getRGBImage()), frame_src.keypoints, 
                    *(frame_dst.getRGBImage()), frame_dst.keypoints, good_matches, img_matches);

    cv::namedWindow("raw matches", CV_WINDOW_NORMAL);
    cv::imshow("raw matches", img_matches);
    cv::waitKey(1);

    if (save)
    {
      std::stringstream ss1;
      ss1 << frame_src.header.seq << "_to_" << frame_dst.header.seq << "_base";
      cv::imwrite("/home/idryanov/ros/images/" + ss1.str() + ".png", img_matches);
    }
  }

  // **** Check if ratio of inliers is high enough

  inlier_ratio = (double)inlier_matches.size() / (double)good_matches.size();
  ransac_overlap = (inlier_ratio > inlier_threshold);

  // ***** compute rigid transformation from inliers

  if (ransac_overlap)
  {
    if (show)
    {
      // drawing & save the inlier matches
      cv::Mat img_inlier_matches;
      cv::drawMatches(*(frame_src.getRGBImage()), frame_src.keypoints, 
                      *(frame_dst.getRGBImage()), frame_dst.keypoints, 
                      inlier_matches, img_inlier_matches);

      cv::namedWindow("inlier matches", CV_WINDOW_NORMAL);
      cv::imshow("inlier matches", img_inlier_matches);
      cv::waitKey(1);

      if (save)
      {
        std::stringstream ss;
        ss << frame_src.header.seq << "_to_" << frame_dst.header.seq << "_inliers";
        cv::imwrite("/home/idryanov/ros/images/" + ss.str() + ".png", img_inlier_matches);
      }
    }

    // create 3D point clouds   
    constructCloudsFromInliers(inlier_matches, frame_src, frame_dst, cloud_src, cloud_dst);

    // estimate using simple svd
    Eigen::Matrix4f transform_eigen;
    pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;
    svd.estimateRigidTransformation(*cloud_src, *cloud_dst, transform_eigen);
    transform = tfFromEigen(transform_eigen);

    // refine estimate using icp
    if (use_icp_refinement_sparse)
    {
      pcl::transformPointCloud(*cloud_src, *cloud_src, eigenFromTf(transform));

      ccny_rgbd::ICPKd<PointT, PointT> reg;
    
      reg.setMaxIterations(20);
      reg.setTransformationEpsilon(0.001);
      reg.setMaxCorrDist(0.15);
      reg.setUseValueRejection(false);
      reg.setUseRANSACRejection(true);
      reg.setRANSACThreshold(0.15);

      pcl::KdTreeFLANN<PointT> tree_data;
      pcl::KdTreeFLANN<PointT> tree_model;

      tree_data.setInputCloud(cloud_src);
      tree_model.setInputCloud(cloud_dst);

      reg.setDataCloud  (&*cloud_src);
      reg.setModelCloud (&*cloud_dst);

      reg.setDataTree  (&tree_data);
      reg.setModelTree (&tree_model);

      reg.align();
      Eigen::Matrix4f icp_corr_eigen = reg.getFinalTransformation();
      tf::Transform icp_corr = tfFromEigen(icp_corr_eigen);

      transform = icp_corr * transform;
    }
  }

  return ransac_overlap;
}


} // namespace ccny_rgbd
