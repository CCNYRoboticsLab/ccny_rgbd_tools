#include "ccny_rgbd/apps/avg_logger.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "AvgLogger");  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::AvgLogger avg_logger(nh, nh_private);
  ros::spin();
  return 0;
}
