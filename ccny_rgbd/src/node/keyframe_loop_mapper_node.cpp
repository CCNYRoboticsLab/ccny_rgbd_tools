#include "ccny_rgbd/apps/keyframe_loop_mapper.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "KeyframeLoopMapper");  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::KeyframeLoopMapper km(nh, nh_private);
  ros::spin();
  return 0;
}
