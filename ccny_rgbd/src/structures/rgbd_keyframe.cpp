#include "ccny_rgbd/structures/rgbd_keyframe.h"

namespace ccny_rgbd
{

RGBDKeyframe::RGBDKeyframe(const RGBDFrame& frame):
  RGBDFrame(frame),
  manually_added(false),
  max_data_range_(5.0),
  max_sigma_z_(0.020) // TODO: Parameter, or max_sigma_z
{
  max_var_z_ = max_sigma_z_ * max_sigma_z_;
}

void RGBDKeyframe::constructDataCloud()
{
  // Use correct principal point from calibration
  float cx = model.cx();
  float cy = model.cy();

  // Scale by focal length for computing (X,Y)
  float constant_x = 1.0 / model.fx();
  float constant_y = 1.0 / model.fy();

  float bad_point = std::numeric_limits<float>::quiet_NaN();

  data.points.clear();
  data.points.resize(rgb_img.rows * rgb_img.cols);
  for (int v = 0; v < rgb_img.rows; ++v)
  for (int u = 0; u < rgb_img.cols; ++u)
  {
    unsigned int index = v * rgb_img.cols + u;

    uint16_t z_raw = depth_img.at<uint16_t>(v, u);
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

    cv::Vec3b& bgr = rgb_img.at<cv::Vec3b>(v,u);
    uint32_t color = (bgr[2] << 16) + (bgr[1] << 8) + bgr[0];
    p.rgb = *reinterpret_cast<float*>(&color);

    data.points[index] = p;
  }

  data.header = header;
  data.height = rgb_img.rows;
  data.width  = rgb_img.cols;
  data.is_dense = false;
}


} // namespace
