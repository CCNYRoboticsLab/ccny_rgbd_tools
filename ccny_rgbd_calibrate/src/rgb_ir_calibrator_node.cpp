#include "ccny_rgbd_calibrate/rgb_ir_calibrator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RGBIRCalibrator");  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::RGBIRCalibrator rgb_ir_calibrator(nh, nh_private);
  return 0;
}
