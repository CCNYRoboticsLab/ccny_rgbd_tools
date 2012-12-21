#include "ccny_rgbd/test_projection.h"
#include "ccny_rgbd/rgbd_util.h"

int main(int argc, char** argv)
{
  ccny_rgbd::test_projection();
  return 0;
}

namespace ccny_rgbd {

void test_projection()
{
 
  // read in
  printf("Reading cloud\n");
  PointCloudT::Ptr cloud;
  cloud.reset(new PointCloudT());
  pcl::PCDReader reader;
  reader.read ("/home/rvalenti/ros/PCDs/office.pcd", cloud);
  
  Matrix3f rmat1; 
  Matrix3f rmat2;
  Matrix3f rmat3;
  Matrix3f rmat;
  rmat2 <<1,0,0,0,1,0,0,0,1 ;
  rmat1 <<0,0,-1,0,1,0,1,0,0;
  rmat3 <<0,-1,0,1,0,0,0,0,1;
  rmat= rmat3 * rmat2* rmat1;
  Vector3f tvec ;
  tvec <<0,0,-1;
  // load intrinsic matrix
  cv::FileStorage fs_mat( "/home/rvalenti/ros/param/intr.yml", cv::FileStorage::READ);

  cv::Mat intr;
  CameraInfoMsg info_msg;
  fs_mat["intr"] >> intr;
  
  cv::Mat rgb_img;
  cv::Mat depth_img;
  cv::Mat filled_rgb_img; 
  cv::Mat filled_depth_img;
  cv::Mat filtered_img;

  Matrix3f intrinsic;
  openCVRToEigenR(intr, intrinsic);
  projectCloudToImage(cloud, rmat, tvec, intrinsic, 320, 240, rgb_img, depth_img);
  holeFilling2(rgb_img, depth_img, 3, filled_rgb_img, filled_depth_img);

  medianBlur(filled_rgb_img,filtered_img, 3);  

  cv::namedWindow("rgb image", 0);
  cv::imshow("rgb image", rgb_img);
  cv::imshow("depth image", (depth_img/20)*255);
  cv::namedWindow("filled rgb image", 0);  
  cv::imshow("filled rgb image", filled_rgb_img);
  cv::namedWindow("filled depth image", 0);
  cv::imshow("filled depth image", (filled_depth_img/20)*255);
  cv::namedWindow("filtered_img", 0);
  cv::imshow("filtered_img", filtered_img);
  
  cv::waitKey(0);
}
}//namespace ccny_rgbd
