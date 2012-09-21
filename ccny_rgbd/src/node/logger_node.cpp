#include "ccny_rgbd/apps/logger.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Logger");  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::Logger logger(nh, nh_private);
  ros::spin();
  return 0;
}
