/**
 *  @file rgbd_keyframe.cpp
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2013, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ccny_rgbd/structures/rgbd_keyframe.h"

namespace ccny_rgbd {

RGBDKeyframe::RGBDKeyframe():
  manually_added(false)
{
 
}

RGBDKeyframe::RGBDKeyframe(const RGBDFrame& frame):
  RGBDFrame(),
  manually_added(false)
{
  rgb_img   = frame.rgb_img.clone();
  depth_img = frame.depth_img.clone();
  header    = frame.header;
  model     = frame.model;
}

bool RGBDKeyframe::save(
  const RGBDKeyframe& keyframe, 
  const std::string& path)
{  
  std::string pose_filename  = path + "/pose.yaml";
  std::string prop_filename  = path + "/properties.yaml"; 

  // save frame  
  bool save_frame_result = RGBDFrame::save(keyframe, path);
  if (!save_frame_result) return false;
  
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

bool RGBDKeyframe::load(RGBDKeyframe& keyframe, const std::string& path)
{
  // load frame
  bool load_frame_result = RGBDFrame::load(keyframe, path);
  if (!load_frame_result) return false;

  // set up filenames
  std::string pose_filename  = path + "/pose.yaml";
  std::string prop_filename  = path + "/properties.yaml"; 

  // check if files exist
  if (!boost::filesystem::exists(pose_filename)  ||
      !boost::filesystem::exists(prop_filename)  )
  {
    ROS_ERROR("files for loading keyframe not found");
    return false;
  }

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

bool saveKeyframes(
  const KeyframeVector& keyframes, 
  const std::string& path)
{
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    std::stringstream ss_idx;
    ss_idx << std::setw(4) << std::setfill('0') << kf_idx;
    
    std::string kf_path = path + "/" + ss_idx.str();

    bool save_result = RGBDKeyframe::save(keyframes[kf_idx], kf_path); 
    if (!save_result) return false;
  }

  return true;
}

bool loadKeyframes(
  KeyframeVector& keyframes, 
  const std::string& path)
{
  keyframes.clear();

  int kf_idx = 0;

  while(true)
  {
    std::stringstream ss_idx;
    ss_idx << std::setw(4) << std::setfill('0') << kf_idx;

    std::string path_kf = path + "/" + ss_idx.str();

    if (boost::filesystem::exists(path_kf))
    {
      ROS_INFO("Loading %s", path_kf.c_str());
      RGBDKeyframe keyframe;
      bool result_load = RGBDKeyframe::load(keyframe, path_kf);
      if (result_load) keyframes.push_back(keyframe);
      else
      {
        ROS_WARN("Error loading"); 
        return false;
      }
    } 
    else return true;

    kf_idx++;
  }
}

} // namespace
