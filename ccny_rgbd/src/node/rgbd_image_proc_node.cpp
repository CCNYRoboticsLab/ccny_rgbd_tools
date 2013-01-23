#include "ccny_rgbd/apps/rgbd_image_proc.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "RGBDImageProc");  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::RGBDImageProc rgbd_image_proc(nh, nh_private);
  ros::spin();
  return 0;
}
