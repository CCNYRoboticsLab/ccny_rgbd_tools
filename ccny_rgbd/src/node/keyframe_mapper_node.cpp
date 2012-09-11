#include "ccny_rgbd/apps/keyframe_mapper.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "KeyframeMapper");  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::KeyframeMapper km(nh, nh_private);
  ros::spin();
  return 0;
}
