#include "ccny_rgbd/apps/octree_mapper.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "OctreeMapper");  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::OctreeMapper om(nh, nh_private);
  ros::spin();
  return 0;
}
