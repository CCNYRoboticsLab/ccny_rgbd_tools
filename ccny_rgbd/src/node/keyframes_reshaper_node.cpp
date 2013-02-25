#include "ccny_rgbd/apps/keyframes_reshaper.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "Reshape Keyframes");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::KeyFramesReshaper reshaper(nh, nh_private);
  ros::spin();
  return 0;
}
