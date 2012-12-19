#include "ccny_rgbd/apps/mono_vo.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "MonocularVisualOdometry");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::MonocularVisualOdometry  mono_vo(nh, nh_private);
  ros::spin();
  return 0;
}
