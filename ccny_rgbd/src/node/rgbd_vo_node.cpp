#include "ccny_rgbd/registration/rgbd_vo.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RGBDVisualOdometry");  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::RGBDVO rgbdvo(nh, nh_private);
  ros::spin();
  return 0;
}
