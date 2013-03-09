#include "ccny_rgbd_calibrate/uncertainty_comparator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "UncertaintyComparator");  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::UncertaintyComparator comparator(nh, nh_private);
  ros::spin();
  return 0;
}
