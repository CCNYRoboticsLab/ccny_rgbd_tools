#include "ccny_rgbd/test_projection.h"
#include "ccny_rgbd/rgbd_util.h"
#include "ccny_rgbd/structures/rgbd_keyframe.h"

int main(int argc, char** argv)
{
  int id = atoi(argv[1]); 
  ccny_rgbd::test_projection(id);
  return 0;
}

namespace ccny_rgbd {

void test_projection(int id)
{
  // params
  int kf_idx = id;
  int w = 640;
  int h = 480;
  std::string keyframes_path_ = "/home/idyanov/ros/office_mc_01_keyframes";
  std::string cloud_filename_ = "/home/idyanov/office_mc_01.pcd";
 
  // path to keyframe
  std::stringstream ss_idx;
  ss_idx << std::setw(4) << std::setfill('0') << kf_idx;
  std::string path_kf = keyframes_path_ + "/" + ss_idx.str();
  
  // read in
  printf("Reading global cloud\n");
  PointCloudT cloud;
  pcl::io::loadPCDFile<PointT>(cloud_filename_, cloud);
  
  // load keyframe
  printf("Loading keyframe: %s\n", path_kf.c_str());
  RGBDKeyframe keyframe;
  RGBDKeyframe::load(keyframe, path_kf);
    
  // determine intrinsic matrix
  Matrix3f intrinsic;
  openCVRToEigenR(keyframe.model.intrinsicMatrix(), intrinsic);
  
  // Rescale intrinsice
  double scale_factor_intrinsic = w / (float) keyframe.rgb_img.cols;
  printf("Scale factor = %f\n", scale_factor_intrinsic);
  intrinsic(0,0) *= scale_factor_intrinsic; // fx
  intrinsic(0,2) *= scale_factor_intrinsic; // cx
  intrinsic(1,1) *= scale_factor_intrinsic; // fy
  intrinsic(1,2) *= scale_factor_intrinsic; // cy
  
  // determine pose
  Matrix3f rmat;
  Vector3f tvec;
  tfToEigenRt(keyframe.pose.inverse(), rmat, tvec);
  
  ros::WallTime start = ros::WallTime::now();
  
  // project
  cv::Mat v_rgb_img, v_depth_img;
  projectCloudToVirtualImage(
    cloud, rmat, tvec, intrinsic, w, h, v_rgb_img, v_depth_img);
  
  printf("%.1f\n", getMsDuration(start));
  
  // filter
  cv::Mat vf_rgb_img;
  
  if (1)
  {
    medianBlur(v_rgb_img, vf_rgb_img, 5);  
    //GaussianBlur(vf_rgb_img, vf_rgb_img, cv::Size(0, 0), 3);
  }
  else
    vf_rgb_img = v_rgb_img;

  printf("%.1f\n", getMsDuration(start));
  
  cv::namedWindow("vf_rgb_img");
  cv::imshow("vf_rgb_img", vf_rgb_img);
 
  cv::namedWindow("v_rgb_img");
  cv::imshow("v_rgb_img", v_rgb_img);
  
  cv::namedWindow("v_depth_img", 0);  
  cv::imshow("v_depth_img", (v_depth_img/20)*255);
  
  cv::namedWindow("kf.rgb_img");
  cv::imshow("kf.rgb_img", keyframe.rgb_img);
 
  cv::namedWindow("kf.depth_img");  
  cv::imshow("kf.depth_img", (keyframe.depth_img/20)*255);

  cv::waitKey(0);
}
}//namespace ccny_rgbd
