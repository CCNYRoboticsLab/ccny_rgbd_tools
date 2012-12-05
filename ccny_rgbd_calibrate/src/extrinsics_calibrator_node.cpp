#include "ccny_rgbd_calibrate/extrinsics_calibrator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ExtrinsicsCalibrator");  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::ExtrinsicsCalibrator extr_calibrator(nh, nh_private);
  return 0;
}
