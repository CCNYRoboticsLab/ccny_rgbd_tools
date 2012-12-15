#include "ccny_rgbd/structures/rgbd_keyframe.h"

namespace ccny_rgbd
{

RGBDKeyframe::RGBDKeyframe(const RGBDFrame& frame):
  RGBDFrame(frame),
  manually_added(false)
{

}

void RGBDKeyframe::constructDensePointCloud(
  double max_z,
  double max_stdev_z)
{
  double max_var_z = max_stdev_z * max_stdev_z; // maximum allowed z variance

  // Use correct principal point from calibration
  float cx = model.cx();
  float cy = model.cy();

  // Scale by focal length for computing (X,Y)
  float constant_x = 1.0 / model.fx();
  float constant_y = 1.0 / model.fy();

  float bad_point = std::numeric_limits<float>::quiet_NaN();

  cloud.points.clear();
  cloud.points.resize(rgb_img.rows * rgb_img.cols);
  for (int v = 0; v < rgb_img.rows; ++v)
  for (int u = 0; u < rgb_img.cols; ++u)
  {
    unsigned int index = v * rgb_img.cols + u;

    uint16_t z_raw = depth_img.at<uint16_t>(v, u);
    float z = z_raw * 0.001; //convert to meters

    PointT& p = cloud.points[index];

    double z_mean, z_var; 

    // check for out of range or bad measurements
    if (z_raw != 0)
    {
      getGaussianMixtureDistribution(u, v, z_mean, z_var);

      // check for variance and z limits     
      if (z_var < max_var_z && z_mean < max_z)
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
 
    // fill out color
    const cv::Vec3b& color = rgb_img.at<cv::Vec3b>(v,u);
    p.r = color[2];
    p.g = color[1];
    p.b = color[0];
  }

  cloud.header = header;
  cloud.height = rgb_img.rows;
  cloud.width  = rgb_img.cols;
  cloud.is_dense = false;
}

bool saveKeyframe(const RGBDKeyframe& keyframe, const std::string& path)
{
  std::string cloud_filename = path + "/cloud.pcd";
  std::string pose_filename  = path + "/pose.yaml";
  std::string prop_filename  = path + "/properties.yaml"; 

  // save frame  
  bool save_frame_result = saveFrame(keyframe, path);
  if (!save_frame_result) return false;
  
  // save cloud as pcd
  pcl::PCDWriter writer;
  int result_pcd = writer.writeBinary<PointT>(cloud_filename, keyframe.cloud);

  if (result_pcd != 0) 
  {
    ROS_ERROR("Error saving point cloud");
    return false;
  }

  // save pose as OpenCV rmat and tvec
  cv::Mat rmat, tvec;
  tfToOpenCVRt(keyframe.pose, rmat, tvec);
  cv::FileStorage fs_m(pose_filename, cv::FileStorage::WRITE);
  fs_m << "rmat" << rmat;
  fs_m << "tvec" << tvec;
 
  // save other class members 
  cv::FileStorage fs_p(prop_filename, cv::FileStorage::WRITE);
  fs_p << "manually_added"      << keyframe.manually_added;
  fs_p << "path_length_linear"  << keyframe.path_length_linear;
  fs_p << "path_length_angular" << keyframe.path_length_angular;

  return true;
}

bool loadKeyframe(RGBDKeyframe& keyframe, const std::string& path)
{
  // load frame
  bool load_frame_result = loadFrame(keyframe, path);
  if (!load_frame_result) return false;

  // set up filenames
  std::string cloud_filename = path + "/cloud.pcd";
  std::string pose_filename  = path + "/pose.yaml";
  std::string prop_filename  = path + "/properties.yaml"; 

  // check if files exist
  if (!boost::filesystem::exists(cloud_filename) ||
      !boost::filesystem::exists(pose_filename)  ||
      !boost::filesystem::exists(prop_filename)  )
  {
    ROS_ERROR("files for loading keyframe not found");
    return false;
  }

  // load cloud from pcd
  pcl::PCDReader reader;
  reader.read (cloud_filename, keyframe.cloud);

  // load pose
  cv::FileStorage fs_m(pose_filename, cv::FileStorage::READ);

  cv::Mat rmat, tvec;
  fs_m["rmat"] >> rmat;
  fs_m["tvec"] >> tvec;

  openCVRtToTf(rmat, tvec, keyframe.pose);

  // load other class members
  cv::FileStorage fs_p(prop_filename, cv::FileStorage::READ);
  fs_p["manually_added"]      >> keyframe.manually_added;
  fs_p["path_length_linear"]  >> keyframe.path_length_linear;
  fs_p["path_length_angular"] >> keyframe.path_length_angular;

  return true;
}

} // namespace
