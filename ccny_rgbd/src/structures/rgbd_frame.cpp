#include "ccny_rgbd/structures/rgbd_frame.h"

namespace ccny_rgbd
{

RGBDFrame::RGBDFrame():
  keypoints_computed(false),
  descriptors_computed(false)
{

}

RGBDFrame::RGBDFrame(const sensor_msgs::ImageConstPtr& rgb_msg,
                     const sensor_msgs::CameraInfoConstPtr& info_msg):
  keypoints_computed(false),
  descriptors_computed(false)  
{
  depth_factor_ = 0.02; // FIXME: proper distortion calibration

  // TODO: Share vs copy?
  // Answer by Carlos: I believe you would prefer to copy here because you are dealing with fast frames whose processing may not be mutually exclussive
  cv_ptr_rgb_   = cv_bridge::toCvCopy(rgb_msg);

  // create camera model
  model_.fromCameraInfo(info_msg);

  // record header - frame is from RGB camera
  header_ = rgb_msg->header;
}



RGBDFrame::RGBDFrame(const sensor_msgs::ImageConstPtr& rgb_msg,
                     const sensor_msgs::ImageConstPtr& depth_msg,
                     const sensor_msgs::CameraInfoConstPtr& info_msg):
  keypoints_computed(false),
  descriptors_computed(false)
{
  depth_factor_ = 0.00; // FIXME: proper distortion calibration

  // TODO: Share vs copy?
  cv_ptr_rgb_   = cv_bridge::toCvCopy(rgb_msg);
  cv_ptr_depth_ = cv_bridge::toCvCopy(depth_msg);

  // create camera model
  model_.fromCameraInfo(info_msg);

  // record header - frame is from RGB camera
  header_ = rgb_msg->header;
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
  double max_var_z = 0.03 * 0.03; // maximum allowed z variance

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
  kp_valid.clear();
  kp_mean.clear();
  kp_covariance.clear();

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

    // ****** FIXME: better distorition model ***********************
    //double factor_s = 1.0 + depth_factor_ * (std::abs(umcx) / 160) + 
    //                        depth_factor_ * (std::abs(vmcy) / 120);
    double factor_s = 1.0;
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

  features.header = header_;
  features.height = 1;
  features.width = features.points.size();
  features.is_dense = true;
}

} // namespace ccny_rgbd
