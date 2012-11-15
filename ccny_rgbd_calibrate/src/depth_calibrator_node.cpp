#include "ccny_rgbd_calibrate/depth_calibrator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "DepthCalibrator");  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::DepthCalibrator depth_calibrator(nh, nh_private);
  return 0;
}
