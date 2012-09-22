#include "ccny_rgbd/structures/rgbd_keyframe.h"

namespace ccny_rgbd
{

RGBDKeyframe::RGBDKeyframe(const RGBDFrame& frame):
  RGBDFrame(frame),
  max_data_range_(5.0),
  max_sigma_z_(0.035) // TODO: Parameter, or max_sigma_z
{
  max_var_z_ = max_sigma_z_ * max_sigma_z_;
}

void RGBDKeyframe::constructDataCloud()
{
  // Use correct principal point from calibration
  float cx = model_.cx();
  float cy = model_.cy();

  // Scale by focal length for computing (X,Y)
  float constant_x = 1.0 / model_.fx();
  float constant_y = 1.0 / model_.fy();

  float bad_point = std::numeric_limits<float>::quiet_NaN();

  data.points.clear();
  data.points.resize(cv_ptr_rgb_->image.rows * cv_ptr_rgb_->image.cols);
  for (int v = 0; v < cv_ptr_rgb_->image.rows; ++v)
  for (int u = 0; u < cv_ptr_rgb_->image.cols; ++u)
  {
    unsigned int index = v * cv_ptr_rgb_->image.cols + u;

    uint16_t z_raw = cv_ptr_depth_->image.at<uint16_t>(v, u);
    float z = z_raw * 0.001; //convert to meters

    PointT p;

    double z_mean, z_var; 

    // check for out of range or bad measurements
    //if (z_raw != 0 && z <= max_data_range_)
    if (z_raw != 0)
    {
      getGaussianMixtureDistribution(u, v, z_mean, z_var);
     
      if (z_var < max_var_z_)
      {
        // fill in XYZ
        p.x = z * (u - cx) * constant_x;
        p.y = z * (v - cy) * constant_y;
        p.z = z;

        // ****** FIXME: better distortion model
        double factor_s = 1.0 + depth_factor_ * (std::abs(u - cx) / 160) + 
                                depth_factor_ * (std::abs(v - cy) / 120);
        p.z = z * factor_s;
        // **************************************************************
      }
      else
      {
        p.x = p.y = p.z = bad_point;
      }
    }
    else
    {
      p.x = p.y = p.z = bad_point;
    }

    cv::Vec3b& bgr = cv_ptr_rgb_->image.at<cv::Vec3b>(v,u);
    uint32_t color = (bgr[2] << 16) + (bgr[1] << 8) + bgr[0];
    p.rgb = *reinterpret_cast<float*>(&color);

    data.points[index] = p;
  }

  data.header = cv_ptr_rgb_->header;
  data.height = cv_ptr_rgb_->image.rows;
  data.width  = cv_ptr_rgb_->image.cols;
  data.is_dense = false;
}


} // namespace
