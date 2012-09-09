#include "ccny_rgbd/apps/visual_odometry.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "VisualOdometry");  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::VisualOdometry vo(nh, nh_private);
  ros::spin();
  return 0;
}
